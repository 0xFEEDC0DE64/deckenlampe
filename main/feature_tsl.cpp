#include "feature_tsl.h"

// esp-idf includes
#include <esp_log.h>

// 3rdparty lib includes
#include <fmt/core.h>
#include <Adafruit_TSL2561_U.h>

// local includes
#include "myconfig.h"
#include "mymqtt.h"
#include "delayedconstruction.h"

using namespace std::chrono_literals;

namespace deckenlampe {
std::optional<TslValue> lastTslValue;
espchrono::millis_clock::time_point last_tsl2561_lux_pub;
bool tslInitialized{};

namespace {
constexpr const char * const TAG = "TSL";

cpputils::DelayedConstruction<Adafruit_TSL2561_Unified> tsl;
espchrono::millis_clock::time_point last_tsl2561_readout;
} // namespace

void init_tsl()
{
    if (!config::enable_i2c.value() || !config::enable_tsl.value())
        return;

    tsl.construct(TSL2561_ADDR_FLOAT, 12345);

    ESP_LOGI(TAG, "calling tsl.begin()...");
    tslInitialized = tsl->begin(true);
    ESP_LOGI(TAG, "finished with %s", tslInitialized ? "true" : "false");

    if (tslInitialized)
    {
        sensor_t sensor = tsl->getSensor();
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
        // tsl->setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
        // tsl->setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
        tsl->enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */

        /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
        tsl->setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
        // tsl->setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
        // tsl->setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

        /* Update these values depending on what you've set above! */
        ESP_LOGI(TAG, "------------------------------------");
        ESP_LOGI(TAG, "Gain:         Auto");
        ESP_LOGI(TAG, "Timing:       13 ms");
        ESP_LOGI(TAG, "------------------------------------");
    }
}

void update_tsl()
{
    if (!config::enable_i2c.value() || !config::enable_tsl.value())
        return;

    if (tslInitialized)
    {
        if (espchrono::ago(last_tsl2561_readout) < 5s)
            return;

        last_tsl2561_readout = espchrono::millis_clock::now();

        if (std::optional<sensors_event_t> event = tsl->getEvent())
        {
            /* Display the results (light is measured in lux) */
            if (event->light)
            {
                TslValue tslValue {
                    .timestamp = espchrono::millis_clock::now(),
                    .lux = event->light
                };
                ESP_LOGI(TAG, "read tsl: %.1f lux", tslValue.lux);

                if (mqttConnected)
                {
                    if (!lastTslValue)
                        mqttVerbosePub(config::topic_tsl2561_availability.value(), "online", 0, 1);
                    if (!lastTslValue || espchrono::ago(last_tsl2561_lux_pub) >= config::valueUpdateInterval.value())
                    {
                        if (mqttVerbosePub(config::topic_tsl2561_lux.value(), fmt::format("{:.1f}", tslValue.lux), 0, 1) >= 0)
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
        if (lastTslValue && espchrono::ago(lastTslValue->timestamp) >= config::availableTimeoutTime.value())
        {
            ESP_LOGW(TAG, "tsl timeouted");
            if (mqttConnected)
                mqttVerbosePub(config::topic_tsl2561_availability.value(), "offline", 0, 1);
            lastTslValue = std::nullopt;
        }
    }
}
} // namespace deckenlampe
