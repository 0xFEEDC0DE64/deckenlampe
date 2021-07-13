#include "sdkconfig.h"

// system includes
#include <chrono>
#include <atomic>

// esp-idf includes
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#if defined(CONFIG_ESP_TASK_WDT_PANIC) || defined(CONFIG_ESP_TASK_WDT)
#include <freertos/task.h>
#include <esp_task_wdt.h>
#endif
#ifdef CONFIG_APP_ROLLBACK_ENABLE
#include <esp_ota_ops.h>
#endif
#include <nvs_flash.h>
#include <esp_http_server.h>
#include <mdns.h>
#include <esp_system.h>
#include <hal/gpio_types.h>

// Arduino includes
#include <Arduino.h>
#include <Wire.h>

// 3rdparty lib includes
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_BMP085_U.h>
#include <DHT_U.h>
#include <fmt/core.h>

// local includes
#include "espwifistack.h"
#include "tickchrono.h"
#include "wrappers/mqtt_client.h"

using namespace std::chrono_literals;

namespace {
constexpr const char * const TAG = "MAIN";

constexpr const gpio_num_t pins_sda = GPIO_NUM_16;
constexpr const gpio_num_t pins_scl = GPIO_NUM_17;
constexpr const gpio_num_t pins_dht = GPIO_NUM_33;

constexpr const std::string_view broker_url = "mqtt://192.168.0.2/";

constexpr const std::string_view topic_lamp_availability = "dahoam/wohnzimmer/deckenlicht1/available";
constexpr const std::string_view topic_lamp_status = "dahoam/wohnzimmer/deckenlicht1/status";
constexpr const std::string_view topic_lamp_set = "dahoam/wohnzimmer/deckenlicht1/set";

constexpr const std::string_view topic_switch_availability = "dahoam/wohnzimmer/lichtschalter1/available";
constexpr const std::string_view topic_switch_status = "dahoam/wohnzimmer/lichtschalter1/status";

constexpr const std::string_view topic_tsl2561_availability = "dahoam/wohnzimmer/tsl2561_1/available";
constexpr const std::string_view topic_tsl2561_lux = "dahoam/wohnzimmer/tsl2561_1/lux";

constexpr const std::string_view topic_bmp085_availability = "dahoam/wohnzimmer/bmp085_1/available";
constexpr const std::string_view topic_bmp085_pressure = "dahoam/wohnzimmer/bmp085_1/pressure";
constexpr const std::string_view topic_bmp085_temperature = "dahoam/wohnzimmer/bmp085_1/temperature";
constexpr const std::string_view topic_bmp085_altitude = "dahoam/wohnzimmer/bmp085_1/altitude";

constexpr const std::string_view topic_dht11_availability = "dahoam/wohnzimmer/dht11_1/available";
constexpr const std::string_view topic_dht11_temperature = "dahoam/wohnzimmer/dht11_1/temperature";
constexpr const std::string_view topic_dht11_humidity = "dahoam/wohnzimmer/dht11_1/humidity";

constexpr const auto availableTimeoutTime = 1min;
constexpr const auto valueUpdateInterval = 15s;



std::atomic<bool> mqttConnected;
espcpputils::mqtt_client mqttClient;

std::atomic<bool> lampState;
std::atomic<bool> switchState;

//espchrono::millis_clock::time_point lastLampToggle;
//espchrono::millis_clock::time_point lastSwitchToggle;


Adafruit_TSL2561_Unified tsl{TSL2561_ADDR_FLOAT, 12345};
bool tslInitialized{};
struct TslValue
{
    espchrono::millis_clock::time_point timestamp;
    float lux;
};
std::optional<TslValue> lastTslValue;
espchrono::millis_clock::time_point last_tsl2561_lux_pub;


Adafruit_BMP085_Unified bmp{10085};
bool bmpInitialized{};
struct BmpValue
{
    espchrono::millis_clock::time_point timestamp;
    float pressure;
    float temperature;
    float altitude;
};
std::optional<BmpValue> lastBmpValue;
espchrono::millis_clock::time_point last_bmp085_pressure_pub;
espchrono::millis_clock::time_point last_bmp085_temperature_pub;
espchrono::millis_clock::time_point last_bmp085_altitude_pub;


DHT_Unified dht(pins_dht, DHT11);
struct DhtValue
{
    espchrono::millis_clock::time_point timestamp;
    float temperature;
    float humidity;
};
std::optional<DhtValue> lastDhtValue;
espchrono::millis_clock::time_point last_dht11_temperature_pub;
espchrono::millis_clock::time_point last_dht11_humidity_pub;



esp_err_t webserver_root_handler(httpd_req_t *req);
esp_err_t webserver_on_handler(httpd_req_t *req);
esp_err_t webserver_off_handler(httpd_req_t *req);
esp_err_t webserver_toggle_handler(httpd_req_t *req);
esp_err_t webserver_reboot_handler(httpd_req_t *req);

void mqttEventHandler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

int mqttVerbosePub(std::string_view topic, std::string_view value, int qos, int retain);
} // namespace

extern "C" void app_main()
{
#if defined(CONFIG_ESP_TASK_WDT_PANIC) || defined(CONFIG_ESP_TASK_WDT)
    {
        const auto taskHandle = xTaskGetCurrentTaskHandle();
        if (!taskHandle)
        {
            ESP_LOGE(TAG, "could not get handle to current main task!");
        }
        else if (const auto result = esp_task_wdt_add(taskHandle); result != ESP_OK)
        {
            ESP_LOGE(TAG, "could not add main task to watchdog: %s", esp_err_to_name(result));
        }
    }
#endif

#ifdef CONFIG_APP_ROLLBACK_ENABLE
    esp_ota_img_states_t ota_state;
    if (const esp_partition_t * const running = esp_ota_get_running_partition())
    {
        if (const auto result = esp_ota_get_state_partition(running, &ota_state); result == ESP_ERR_NOT_FOUND)
            ota_state = ESP_OTA_IMG_VALID;
        else if (result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_ota_get_state_partition() failed with %s", esp_err_to_name(result));
            ota_state = ESP_OTA_IMG_UNDEFINED;
        }
    }
    else
    {
        ESP_LOGE(TAG, "esp_ota_get_running_partition() returned nullptr");
        ota_state = ESP_OTA_IMG_UNDEFINED;
    }
#endif

    {
        const auto result = nvs_flash_init();
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "nvs_flash_init(): %s", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    nvs_handle_t nvsHandle;

    {
        const auto result = nvs_open("deckenlampe", NVS_READWRITE, &nvsHandle);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "nvs_open(): %s", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    wifi_stack::config wifiConfig {
        .wifiEnabled = true,
        .hostname = "deckenlampe1",
        .wifis = std::array<wifi_stack::wifi_entry, 10> {
            wifi_stack::wifi_entry { .ssid = "McDonalds Free WiFi", .key = "Passwort_123" },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} }
        },
        .sta_ip = {
            .dhcpEnabled = true,
//            .staticIp = {},
//            .staticGateway = {},
//            .staticSubnet = {},
//            .staticDns1 = {},
//            .staticDns2 = {}
        },
        .ap = {
            {
                .ssid = "deckenlampe1",
                .key = "Passwort_123"
            },
            .channel = 1,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .ssid_hidden = false,
            .max_connection = 4,
            .beacon_interval = 100,
            .ip{192, 168, 4, 1},
            .subnet{255, 255, 255, 0}
        },
        .min_rssi = -90
    };

    wifi_stack::init(wifiConfig);

    httpd_handle_t httpdHandle{};

    {
        httpd_config_t httpConfig HTTPD_DEFAULT_CONFIG();

        const auto result = httpd_start(&httpdHandle, &httpConfig);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "httpd_start(): %s", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    {
        httpd_uri_t uri {
            .uri = "/",
            .method = HTTP_GET,
            .handler = webserver_root_handler,
            .user_ctx = NULL
        };

        const auto result = httpd_register_uri_handler(httpdHandle, &uri);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "httpd_register_uri_handler() for %s: %s", "/", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    {
        httpd_uri_t uri {
            .uri = "/on",
            .method = HTTP_GET,
            .handler = webserver_on_handler,
            .user_ctx = NULL
        };

        const auto result = httpd_register_uri_handler(httpdHandle, &uri);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "httpd_register_uri_handler() for %s: %s", "/on", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    {
        httpd_uri_t uri {
            .uri = "/off",
            .method = HTTP_GET,
            .handler = webserver_off_handler,
            .user_ctx = NULL
        };

        const auto result = httpd_register_uri_handler(httpdHandle, &uri);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "httpd_register_uri_handler() for %s: %s", "/off", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    {
        httpd_uri_t uri {
            .uri = "/toggle",
            .method = HTTP_GET,
            .handler = webserver_toggle_handler,
            .user_ctx = NULL
        };

        const auto result = httpd_register_uri_handler(httpdHandle, &uri);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "httpd_register_uri_handler() for %s: %s", "/toggle", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    {
        httpd_uri_t uri {
            .uri = "/reboot",
            .method = HTTP_GET,
            .handler = webserver_reboot_handler,
            .user_ctx = NULL
        };

        const auto result = httpd_register_uri_handler(httpdHandle, &uri);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "httpd_register_uri_handler() for %s: %s", "/reboot", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    {
        const auto result = mdns_init();
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "mdns_init(): %s", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    {
        const auto result = mdns_hostname_set(wifiConfig.hostname.c_str());
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "mdns_hostname_set(): %s", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    {
        const auto result = mdns_instance_name_set(wifiConfig.hostname.c_str());
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "mdns_instance_name_set(): %s", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    {
        const auto result = mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "mdns_service_add(): %s", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    {
        esp_mqtt_client_config_t mqtt_cfg = {
            .uri = broker_url.data(),
        };

        mqttClient = espcpputils::mqtt_client{&mqtt_cfg};

        if (!mqttClient)
        {
            ESP_LOGE(TAG, "error while initializing mqtt client!");
            return;
        }
        else
        {
            {
                const auto result = mqttClient.register_event(MQTT_EVENT_ANY, mqttEventHandler, NULL);
                ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "esp_mqtt_client_register_event(): %s", esp_err_to_name(result));
                //if (result != ESP_OK)
                //    return result;
            }

            const auto result = mqttClient.start();
            ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "esp_mqtt_client_start(): %s", esp_err_to_name(result));
            //if (result != ESP_OK)
            //    return result;
        }
    }

    {
        ESP_LOGI(TAG, "calling Wire.begin()...");
        const auto result = Wire.begin(pins_sda, pins_scl);
        ESP_LOGI(TAG, "finished with %s", result ? "true" : "false");
    }

    {
        ESP_LOGI(TAG, "calling tsl.begin()...");
        tslInitialized = tsl.begin(true);
        ESP_LOGI(TAG, "finished with %s", tslInitialized ? "true" : "false");

        if (tslInitialized)
        {
            sensor_t sensor = tsl.getSensor();
            ESP_LOGI(TAG, "------------------------------------");
            ESP_LOGI(TAG, "Sensor:       %s", sensor.name);
            ESP_LOGI(TAG, "Driver Ver:   %i", sensor.version);
            ESP_LOGI(TAG, "Unique ID:    %i", sensor.sensor_id);
            ESP_LOGI(TAG, "Max Value:    %.1f lux", sensor.max_value);
            ESP_LOGI(TAG, "Min Value:    %.1f lux", sensor.min_value);
            ESP_LOGI(TAG, "Resolution:   %.1f lux", sensor.resolution);
            ESP_LOGI(TAG, "------------------------------------");
            ESP_LOGI(TAG, "");

            /* You can also manually set the gain or enable auto-gain support */
            // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
            // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
            tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

            /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
            tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
            // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
            // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

            /* Update these values depending on what you've set above! */
            ESP_LOGI(TAG, "------------------------------------");
            ESP_LOGI(TAG, "Gain:         Auto");
            ESP_LOGI(TAG, "Timing:       13 ms");
            ESP_LOGI(TAG, "------------------------------------");
        }
    }

    {
        ESP_LOGI(TAG, "calling bmp.begin()...");
        bmpInitialized = bmp.begin();
        ESP_LOGI(TAG, "finished with %s", bmpInitialized ? "true" : "false");

        if (bmpInitialized)
        {
            sensor_t sensor = bmp.getSensor();
            ESP_LOGI(TAG, "------------------------------------");
            ESP_LOGI(TAG, "Sensor:       %s", sensor.name);
            ESP_LOGI(TAG, "Driver Ver:   %i", sensor.version);
            ESP_LOGI(TAG, "Unique ID:    %i", sensor.sensor_id);
            ESP_LOGI(TAG, "Max Value:    %.1f hPa", sensor.max_value);
            ESP_LOGI(TAG, "Min Value:    %.1f hPa", sensor.min_value);
            ESP_LOGI(TAG, "Resolution:   %.1f hPa", sensor.resolution);
            ESP_LOGI(TAG, "------------------------------------");
            ESP_LOGI(TAG, "");
        }
    }

    {
        ESP_LOGI(TAG, "calling dht.begin()...");
        dht.begin();
        ESP_LOGI(TAG, "finished");

        {
            sensor_t sensor = dht.temperature().getSensor();
            ESP_LOGI(TAG, "------------------------------------");
            ESP_LOGI(TAG, "Temperature Sensor");
            ESP_LOGI(TAG, "Sensor:       %s", sensor.name);
            ESP_LOGI(TAG, "Driver Ver:   %i", sensor.version);
            ESP_LOGI(TAG, "Unique ID:    %i", sensor.sensor_id);
            ESP_LOGI(TAG, "Max Value:    %.1f C", sensor.max_value);
            ESP_LOGI(TAG, "Min Value:    %.1f C", sensor.min_value);
            ESP_LOGI(TAG, "Resolution:   %.1f C", sensor.resolution);
            ESP_LOGI(TAG, "------------------------------------");
            ESP_LOGI(TAG, "");
        }

        {
            sensor_t sensor = dht.humidity().getSensor();
            ESP_LOGI(TAG, "------------------------------------");
            ESP_LOGI(TAG, "Humidity Sensor");
            ESP_LOGI(TAG, "Sensor:       %s", sensor.name);
            ESP_LOGI(TAG, "Driver Ver:   %i", sensor.version);
            ESP_LOGI(TAG, "Unique ID:    %i", sensor.sensor_id);
            ESP_LOGI(TAG, "Max Value:    %.1f %%", sensor.max_value);
            ESP_LOGI(TAG, "Min Value:    %.1f %%", sensor.min_value);
            ESP_LOGI(TAG, "Resolution:   %.1f %%", sensor.resolution);
            ESP_LOGI(TAG, "------------------------------------");
            ESP_LOGI(TAG, "");
        }
    }

    while (true)
    {
        wifi_stack::update(wifiConfig);

//        if (espchrono::ago(lastLampToggle) >= 10s)
//        {
//            lastLampToggle = espchrono::millis_clock::now();
//            lampState = !lampState;

//            if (mqttClient && mqttConnected)
//                mqttVerbosePub(topic_lamp_status, lampState ? "ON" : "OFF", 0, 1);
//        }

//        if (espchrono::ago(lastSwitchToggle) >= 30s)
//        {
//            lastSwitchToggle = espchrono::millis_clock::now();
//            switchState = !switchState;

//            if (mqttClient && mqttConnected)
//                mqttVerbosePub(topic_switch_status, switchState ? "ON" : "OFF", 0, 1);
//        }

        if (tslInitialized)
        {
            if (std::optional<sensors_event_t> event = tsl.getEvent())
            {
                /* Display the results (light is measured in lux) */
                if (event->light)
                {
                    TslValue tslValue {
                        .timestamp = espchrono::millis_clock::now(),
                        .lux = event->light
                    };
                    ESP_LOGI(TAG, "read tsl: %.1f lux", tslValue.lux);

                    if (mqttClient && mqttConnected)
                    {
                        if (!lastTslValue)
                            mqttVerbosePub(topic_tsl2561_availability, "online", 0, 1);
                        if (!lastTslValue || espchrono::ago(last_tsl2561_lux_pub) >= valueUpdateInterval)
                        {
                            if (mqttVerbosePub(topic_tsl2561_lux, fmt::format("{:.1f}", tslValue.lux), 0, 1) >= 0)
                                last_tsl2561_lux_pub = espchrono::millis_clock::now();
                        }
                    }

                    lastTslValue = tslValue;
                }
                else
                {
                    /* If event.light = 0 lux the sensor is probably saturated
                     * and no reliable data could be generated! */
                    ESP_LOGW(TAG, "tsl sensor overload %f", event->light);
                    goto tslOffline;
                }
            }
            else
            {
                ESP_LOGW(TAG, "tsl failed");
                goto tslOffline;
            }
        }
        else
        {
            tslOffline:
            if (lastTslValue && espchrono::ago(lastTslValue->timestamp) >= availableTimeoutTime)
            {
                ESP_LOGW(TAG, "tsl timeouted");
                if (mqttClient && mqttConnected)
                    mqttVerbosePub(topic_tsl2561_availability, "offline", 0, 1);
                lastTslValue = std::nullopt;
            }
        }

        if (bmpInitialized)
        {
            if (std::optional<Adafruit_BMP085_Unified::TemperatureAndPressure> values = bmp.getTemperatureAndPressure())
            {
                if (values->temperature && values->pressure)
                {
                    BmpValue bmpValue {
                        .timestamp = espchrono::millis_clock::now(),
                        .pressure = values->pressure,
                        .temperature = values->temperature
                    };
                    ESP_LOGI(TAG, "read bmp Pressure: %.1f hPa", bmpValue.pressure);
                    ESP_LOGI(TAG, "read bmp Temperature: %.1f C", bmpValue.temperature);

                    /* Then convert the atmospheric pressure, and SLP to altitude         */
                    /* Update this next line with the current SLP for better results      */
                    constexpr const float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
                    bmpValue.altitude = bmp.pressureToAltitude(seaLevelPressure, bmpValue.pressure);
                    ESP_LOGI(TAG, "read bmp Altitude: %.1f m", bmpValue.altitude);

                    if (mqttClient && mqttConnected)
                    {
                        if (!lastBmpValue)
                            mqttVerbosePub(topic_bmp085_availability, "online", 0, 1);
                        if (!lastBmpValue || espchrono::ago(last_bmp085_pressure_pub) >= valueUpdateInterval)
                        {
                            if (mqttVerbosePub(topic_bmp085_pressure, fmt::format("{:.1f}", bmpValue.pressure), 0, 1) >= 0)
                                last_bmp085_pressure_pub = espchrono::millis_clock::now();
                        }
                        if (!lastBmpValue || espchrono::ago(last_bmp085_temperature_pub) >= valueUpdateInterval)
                        {
                            if (mqttVerbosePub(topic_bmp085_temperature, fmt::format("{:.1f}", bmpValue.temperature), 0, 1) >= 0)
                                last_bmp085_temperature_pub = espchrono::millis_clock::now();
                        }
                        if (!lastBmpValue || espchrono::ago(last_bmp085_altitude_pub) >= valueUpdateInterval)
                        {
                            if (mqttVerbosePub(topic_bmp085_altitude, fmt::format("{:.1f}", bmpValue.altitude), 0, 1) >= 0)
                                last_bmp085_altitude_pub = espchrono::millis_clock::now();
                        }
                    }

                    lastBmpValue = bmpValue;
                }
                else
                {
                    ESP_LOGW(TAG, "bmp sensor error");
                    goto bmpOffline;
                }
            }
            else
            {
                ESP_LOGW(TAG, "bmp failed");
                goto bmpOffline;
            }
        }
        else
        {
            bmpOffline:
            if (lastBmpValue && espchrono::ago(lastBmpValue->timestamp) >= availableTimeoutTime)
            {
                ESP_LOGW(TAG, "bmp timeouted");
                if (mqttClient && mqttConnected)
                    mqttVerbosePub(topic_bmp085_availability, "offline", 0, 1);
                lastBmpValue = std::nullopt;
            }
        }

        {
            std::optional<sensors_event_t> temperatureEvent = dht.temperature().getEvent();
            std::optional<sensors_event_t> humidityEvent = dht.humidity().getEvent();

            if (temperatureEvent && humidityEvent)
            {
                DhtValue dhtValue {
                    .timestamp = espchrono::millis_clock::now(),
                    .temperature = temperatureEvent->temperature,
                    .humidity = humidityEvent->relative_humidity
                };

                ESP_LOGI(TAG, "read dht temperature: %.1f C", dhtValue.temperature);
                ESP_LOGI(TAG, "read dht humidity: %.1f %%", dhtValue.humidity);

                if (mqttClient && mqttConnected)
                {
                    if (!lastDhtValue)
                        mqttVerbosePub(topic_dht11_availability, "online", 0, 1);
                    if (!lastDhtValue || espchrono::ago(last_dht11_temperature_pub) >= valueUpdateInterval)
                    {
                        if (mqttVerbosePub(topic_dht11_temperature, fmt::format("{:.1f}", dhtValue.temperature), 0, 1) >= 0)
                            last_dht11_temperature_pub = espchrono::millis_clock::now();
                    }
                    if (!lastDhtValue || espchrono::ago(last_dht11_humidity_pub) >= valueUpdateInterval)
                    {
                        if (mqttVerbosePub(topic_dht11_humidity, fmt::format("{:.1f}", dhtValue.humidity), 0, 1) >= 0)
                            last_dht11_humidity_pub = espchrono::millis_clock::now();
                    }
                }

                lastDhtValue = dhtValue;
            }
            else
            {
                ESP_LOGW(TAG, "dht temperature or humidity failed");
                if (lastDhtValue && espchrono::ago(lastDhtValue->timestamp) >= availableTimeoutTime)
                {
                    ESP_LOGW(TAG, "dht timeouted");
                    if (mqttClient && mqttConnected)
                        mqttVerbosePub(topic_dht11_availability, "offline", 0, 1);
                    lastDhtValue = std::nullopt;
                }
            }
        }

#if defined(CONFIG_ESP_TASK_WDT_PANIC) || defined(CONFIG_ESP_TASK_WDT)
        if (const auto result = esp_task_wdt_reset(); result != ESP_OK)
            ESP_LOGE(TAG, "esp_task_wdt_reset() failed with %s", esp_err_to_name(result));
#endif

#ifdef CONFIG_APP_ROLLBACK_ENABLE
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            const auto result = esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "esp_ota_mark_app_valid_cancel_rollback() returned: %s", esp_err_to_name(result));
            ota_state = ESP_OTA_IMG_VALID;
        }
#endif

        vPortYield();

        vTaskDelay(std::chrono::ceil<espcpputils::ticks>(2000ms).count());
    }
}

#define CALL_AND_EXIT_ON_ERROR(METHOD, ...) \
    if (const auto result = METHOD(__VA_ARGS__); result != ESP_OK) \
    { \
        ESP_LOGE(TAG, "%s() failed with %s", #METHOD, esp_err_to_name(result)); \
        return result; \
    }

#define CALL_AND_EXIT(METHOD, ...) \
    if (const auto result = METHOD(__VA_ARGS__); result != ESP_OK) \
    { \
        ESP_LOGE(TAG, "%s() failed with %s", #METHOD, esp_err_to_name(result)); \
        return result; \
    } \
    else \
        return result;

namespace {
esp_err_t webserver_root_handler(httpd_req_t *req)
{
    std::string body = "this is work in progress...<br/>\n"
                       "<a href=\"/on\">on</a><br/>\n"
                       "<a href=\"/off\">off</a><br/>\n"
                       "<a href=\"/toggle\">toggle</a><br/>\n"
                       "<a href=\"/reboot\">reboot</a><br/>\n"
                       "<br/>\n";

    body += fmt::format("Lamp: {}<br/>\n", lampState ? "ON" : "OFF");
    body += fmt::format("Switch: {}<br/>\n", switchState ? "ON" : "OFF");

    if (lastTslValue)
    {
        body += fmt::format("Brightness: {:.1f} lux<br/>\n", lastTslValue->lux);
    }

    if (lastBmpValue)
    {
        body += fmt::format("BMP085 Pressure: {:.1f} lux<br/>\n", lastBmpValue->pressure);
        body += fmt::format("BMP085 Temperature: {:.1f} C<br/>\n", lastBmpValue->temperature);
        body += fmt::format("BMP085 Altitude: {:.1f} m<br/>\n", lastBmpValue->altitude);
    }

    if (lastDhtValue)
    {
        body += fmt::format("DHT11 Temperature: {:.1f} C<br/>\n", lastDhtValue->temperature);
        body += fmt::format("DHT11 Humidity: {:.1f} %<br/>\n", lastDhtValue->humidity);
    }

    CALL_AND_EXIT_ON_ERROR(httpd_resp_set_type, req, "text/html")
    CALL_AND_EXIT_ON_ERROR(httpd_resp_send, req, body.data(), body.size())

    return ESP_OK;
}

esp_err_t webserver_on_handler(httpd_req_t *req)
{
    const bool state = (lampState = true);

    if (mqttClient && mqttConnected)
        mqttVerbosePub(topic_lamp_status, state ? "ON" : "OFF", 0, 1);

    std::string_view body{"ON called..."};
    CALL_AND_EXIT_ON_ERROR(httpd_resp_set_type, req, "text/html")
    CALL_AND_EXIT_ON_ERROR(httpd_resp_send, req, body.data(), body.size())

    return ESP_OK;
}

esp_err_t webserver_off_handler(httpd_req_t *req)
{
    const bool state = (lampState = false);

    if (mqttClient && mqttConnected)
        mqttVerbosePub(topic_lamp_status, state ? "ON" : "OFF", 0, 1);

    std::string_view body{"OFF called..."};
    CALL_AND_EXIT_ON_ERROR(httpd_resp_set_type, req, "text/html")
    CALL_AND_EXIT_ON_ERROR(httpd_resp_send, req, body.data(), body.size())

    return ESP_OK;
}

esp_err_t webserver_toggle_handler(httpd_req_t *req)
{
    const bool state = (lampState = !lampState);

    if (mqttClient && mqttConnected)
        mqttVerbosePub(topic_lamp_status, state ? "ON" : "OFF", 0, 1);

    std::string_view body{"TOGGLE called..."};
    CALL_AND_EXIT_ON_ERROR(httpd_resp_set_type, req, "text/html")
    CALL_AND_EXIT_ON_ERROR(httpd_resp_send, req, body.data(), body.size())

    return ESP_OK;
}

esp_err_t webserver_reboot_handler(httpd_req_t *req)
{
    std::string_view body{"REBOOT called..."};
    CALL_AND_EXIT_ON_ERROR(httpd_resp_set_type, req, "text/html")
    CALL_AND_EXIT_ON_ERROR(httpd_resp_send, req, body.data(), body.size())

    esp_restart();

    return ESP_OK;
}

void mqttEventHandler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    CPP_UNUSED(event_handler_arg)

    const esp_mqtt_event_t *data = reinterpret_cast<const esp_mqtt_event_t *>(event_data);

    switch (esp_mqtt_event_id_t(event_id))
    {
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_ERROR");

        //        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        //        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        //        {
        //            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
        //            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
        //            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
        //            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        //        }
        break;

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_CONNECTED");

        mqttConnected = true;

        mqttVerbosePub(topic_lamp_availability, "online", 0, 1);
        mqttVerbosePub(topic_lamp_status, lampState.load() ? "ON" : "OFF", 0, 1);

        mqttVerbosePub(topic_switch_availability, "online", 0, 1);
        mqttVerbosePub(topic_switch_status, switchState.load() ? "ON" : "OFF", 0, 1);

        mqttVerbosePub(topic_tsl2561_availability, lastTslValue ? "online" : "offline", 0, 1);
        if (lastTslValue)
        {
            if (mqttVerbosePub(topic_tsl2561_lux, fmt::format("{:.1f}", lastTslValue->lux), 0, 1) >= 0)
                last_tsl2561_lux_pub = espchrono::millis_clock::now();
        }

        mqttVerbosePub(topic_bmp085_availability, lastBmpValue ? "online" : "offline", 0, 1);
        if (lastBmpValue)
        {
            if (mqttVerbosePub(topic_bmp085_pressure, fmt::format("{:.1f}", lastBmpValue->pressure), 0, 1) >= 0)
                last_bmp085_pressure_pub = espchrono::millis_clock::now();
            if (mqttVerbosePub(topic_bmp085_temperature, fmt::format("{:.1f}", lastBmpValue->temperature), 0, 1) >= 0)
                last_bmp085_temperature_pub = espchrono::millis_clock::now();
            if (mqttVerbosePub(topic_bmp085_altitude, fmt::format("{:.1f}", lastBmpValue->altitude), 0, 1) >= 0)
                last_bmp085_altitude_pub = espchrono::millis_clock::now();
        }

        mqttVerbosePub(topic_dht11_availability, lastDhtValue ? "online" : "offline", 0, 1);
        if (lastDhtValue)
        {
            if (mqttVerbosePub(topic_dht11_temperature, fmt::format("{:.1f}", lastDhtValue->temperature), 0, 1) >= 0)
                last_dht11_temperature_pub = espchrono::millis_clock::now();
            if (mqttVerbosePub(topic_dht11_humidity, fmt::format("{:.1f}", lastDhtValue->humidity), 0, 1) >= 0)
                last_dht11_humidity_pub = espchrono::millis_clock::now();
        }

        mqttClient.subscribe(topic_lamp_set, 0);

        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_DISCONNECTED");

        mqttConnected = false;

        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_SUBSCRIBED");
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_UNSUBSCRIBED");
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_PUBLISHED");
        break;

    case MQTT_EVENT_DATA:
    {
        std::string_view topic{data->topic, (size_t)data->topic_len};
        std::string_view value{data->data, (size_t)data->data_len};

        ESP_LOGI(TAG, "%s event_id=%s topic=%.*s data=%.*s", event_base, "MQTT_EVENT_DATA", topic.size(), topic.data(), value.size(), value.data());

        if (topic == topic_lamp_set)
        {
            bool newState = (lampState = (value == "ON"));
            if (mqttClient && mqttConnected)
                mqttVerbosePub(topic_lamp_status, newState ? "ON" : "OFF", 0, 1);
        }

        break;
    }
    case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGI(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_BEFORE_CONNECT");
        break;

    case MQTT_EVENT_DELETED:
        ESP_LOGI(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_DELETED");
        break;

    default:
        ESP_LOGW(TAG, "%s unknown event_id %i", event_base, event_id);
        break;
    }
}

int mqttVerbosePub(std::string_view topic, std::string_view value, int qos, int retain)
{
    ESP_LOGD(TAG, "topic=\"%.*s\" value=\"%.*s\"", topic.size(), topic.data(), value.size(), value.data());
    const auto pending_msg_id = mqttClient.publish(topic, value, qos, retain);
    if (pending_msg_id < 0)
        ESP_LOGE(TAG, "topic=\"%.*s\" value=\"%.*s\" failed pending_msg_id=%i", topic.size(), topic.data(), value.size(), value.data(), pending_msg_id);
    else
        ESP_LOGI(TAG, "topic=\"%.*s\" value=\"%.*s\" succeeded pending_msg_id=%i", topic.size(), topic.data(), value.size(), value.data(), pending_msg_id);
    return pending_msg_id;
}
} // namespace

auto espchrono::local_clock::timezone() noexcept -> time_zone
{
    return time_zone{1h, DayLightSavingMode::EuropeanSummerTime};
}
