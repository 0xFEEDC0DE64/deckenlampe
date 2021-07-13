#include "sdkconfig.h"

// system includes
#include <chrono>

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

// Arduino includes
#include <Arduino.h>

// local includes
#include "espwifistack.h"
#include "tickchrono.h"
#include "wrappers/mqtt_client.h"

using namespace std::chrono_literals;

namespace {
constexpr const char * const TAG = "MAIN";

espcpputils::mqtt_client mqttClient;

bool lightState{};
bool switchState{};

//espchrono::millis_clock::time_point lastLightToggle;
//espchrono::millis_clock::time_point lastSwitchToggle;

esp_err_t webserver_handler(httpd_req_t *req);

void mqttEventHandler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
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
            .handler = webserver_handler,
            .user_ctx = NULL
        };

        const auto result = httpd_register_uri_handler(httpdHandle, &uri);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "httpd_register_uri_handler(): %s", esp_err_to_name(result));
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
            .uri = "mqtt://192.168.0.2/",
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

    while (true)
    {
        wifi_stack::update(wifiConfig);

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

//        if (espchrono::ago(lastLightToggle) >= 10s)
//        {
//            lastLightToggle = espchrono::millis_clock::now();
//            lightState = !lightState;

//            if (mqttClient)
//            {
//                std::string_view topic = "dahoam/wohnzimmer/deckenlicht1/status";
//                std::string_view value = lightState ? "ON" : "OFF";

//                ESP_LOGI(TAG, "sending %.*s = %.*s", topic.size(), topic.data(), value.size(), value.data());
//                mqttClient.publish(topic, value, 0, 1);
//            }
//        }

//        if (espchrono::ago(lastSwitchToggle) >= 30s)
//        {
//            lastSwitchToggle = espchrono::millis_clock::now();
//            switchState = !switchState;

//            if (mqttClient)
//            {
//                std::string_view topic = "dahoam/wohnzimmer/lichtschalter1/status";
//                std::string_view value = switchState ? "ON" : "OFF";

//                ESP_LOGI(TAG, "sending %.*s = %.*s", topic.size(), topic.data(), value.size(), value.data());
//                mqttClient.publish(topic, value, 0, 1);
//            }
//        }

        vPortYield();

        vTaskDelay(std::chrono::ceil<espcpputils::ticks>(100ms).count());
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
esp_err_t webserver_handler(httpd_req_t *req)
{
    std::string_view body{"this is work in progress..."};
    CALL_AND_EXIT_ON_ERROR(httpd_resp_set_type, req, "text/html")
    CALL_AND_EXIT_ON_ERROR(httpd_resp_send, req, body.data(), body.size())

    return ESP_OK;
}

void mqttEventHandler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    CPP_UNUSED(event_handler_arg)

    const esp_mqtt_event_t *data = reinterpret_cast<const esp_mqtt_event_t *>(event_data);

    switch (esp_mqtt_event_id_t(event_id)) {
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_ERROR");

        //        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        //        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
        //            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
        //            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
        //            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
        //            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        //        }
        break;

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_CONNECTED");

        mqttClient.publish("dahoam/wohnzimmer/deckenlicht1/available", "online", 0, 0);
        mqttClient.publish("dahoam/wohnzimmer/deckenlicht1/status", lightState ? "ON" : "OFF", 0, 1);

        mqttClient.publish("dahoam/wohnzimmer/lichtschalter1/available", "online", 0, 0);
        mqttClient.publish("dahoam/wohnzimmer/lichtschalter1/status", switchState ? "ON" : "OFF", 0, 1);

        mqttClient.subscribe("dahoam/wohnzimmer/deckenlicht1/set", 0);

        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_DISCONNECTED");
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

    case MQTT_EVENT_DATA: {
        std::string_view topic{data->topic, (size_t)data->topic_len};
        std::string_view value{data->data, (size_t)data->data_len};

        ESP_LOGI(TAG, "%s event_id=%s topic=%.*s data=%.*s", event_base, "MQTT_EVENT_DATA", topic.size(), topic.data(), value.size(), value.data());

        if (topic == "dahoam/wohnzimmer/deckenlicht1/set")
        {
            lightState = value == "ON";
            if (mqttClient)
            {
                topic = "dahoam/wohnzimmer/deckenlicht1/status";
                value = lightState ? "ON" : "OFF";

                ESP_LOGI(TAG, "sending %.*s = %.*s", topic.size(), topic.data(), value.size(), value.data());
                mqttClient.publish(topic, value, 0, 1);
            }
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
} // namespace
