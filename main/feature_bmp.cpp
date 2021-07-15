#include "feature_bmp.h"

// esp-idf includes
#include <esp_log.h>

// 3rdparty lib includes
#include <fmt/core.h>
#include <Adafruit_BMP085_U.h>

// local includes
#include "myconfig.h"
#include "mymqtt.h"
#include "delayedconstruction.h"

using namespace std::chrono_literals;

namespace deckenlampe {
std::optional<BmpValue> lastBmpValue;
espchrono::millis_clock::time_point last_bmp085_pressure_pub;
espchrono::millis_clock::time_point last_bmp085_temperature_pub;
espchrono::millis_clock::time_point last_bmp085_altitude_pub;
bool bmpInitialized{};

namespace {
constexpr const char * const TAG = "BMP";

cpputils::DelayedConstruction<Adafruit_BMP085_Unified> bmp;
espchrono::millis_clock::time_point last_bmp085_readout;
} // namespace

void init_bmp()
{
    if (!config::enable_bmp)
        return;

    bmp.construct(10085);

    ESP_LOGI(TAG, "calling bmp.begin()...");
    bmpInitialized = bmp->begin();
    ESP_LOGI(TAG, "finished with %s", bmpInitialized ? "true" : "false");

    if (bmpInitialized) {
        sensor_t sensor = bmp->getSensor();
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

void update_bmp()
{
    if (!config::enable_bmp)
        return;

    if (bmpInitialized) {
        if (espchrono::ago(last_bmp085_readout) < 5s)
            return;

        last_bmp085_readout = espchrono::millis_clock::now();

        if (std::optional<Adafruit_BMP085_Unified::TemperatureAndPressure> values = bmp->getTemperatureAndPressure()) {
            if (values->temperature && values->pressure) {
                BmpValue bmpValue {
                    .timestamp = espchrono::millis_clock::now(),
                    .pressure = values->pressure,
                    .temperature = values->temperature};
                ESP_LOGI(TAG, "read bmp Pressure: %.1f hPa", bmpValue.pressure);
                ESP_LOGI(TAG, "read bmp Temperature: %.1f C", bmpValue.temperature);

                /* Then convert the atmospheric pressure, and SLP to altitude         */
                /* Update this next line with the current SLP for better results      */
                constexpr const float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
                bmpValue.altitude = bmp->pressureToAltitude(seaLevelPressure, bmpValue.pressure);
                ESP_LOGI(TAG, "read bmp Altitude: %.1f m", bmpValue.altitude);

                if (mqttClient && mqttConnected) {
                    if (!lastBmpValue)
                        mqttVerbosePub(config::topic_bmp085_availability, "online", 0, 1);
                    if (!lastBmpValue || espchrono::ago(last_bmp085_pressure_pub) >= config::valueUpdateInterval) {
                        if (mqttVerbosePub(config::topic_bmp085_pressure, fmt::format("{:.1f}", bmpValue.pressure), 0, 1) >= 0)
                            last_bmp085_pressure_pub = espchrono::millis_clock::now();
                    }
                    if (!lastBmpValue || espchrono::ago(last_bmp085_temperature_pub) >= config::valueUpdateInterval) {
                        if (mqttVerbosePub(config::topic_bmp085_temperature, fmt::format("{:.1f}", bmpValue.temperature), 0, 1) >= 0)
                            last_bmp085_temperature_pub = espchrono::millis_clock::now();
                    }
                    if (!lastBmpValue || espchrono::ago(last_bmp085_altitude_pub) >= config::valueUpdateInterval) {
                        if (mqttVerbosePub(config::topic_bmp085_altitude, fmt::format("{:.1f}", bmpValue.altitude), 0, 1) >= 0)
                            last_bmp085_altitude_pub = espchrono::millis_clock::now();
                    }
                }

                lastBmpValue = bmpValue;
            } else {
                ESP_LOGW(TAG, "bmp sensor error");
                goto bmpOffline;
            }
        } else {
            ESP_LOGW(TAG, "bmp failed");
            goto bmpOffline;
        }
    } else {
    bmpOffline:
        if (lastBmpValue && espchrono::ago(lastBmpValue->timestamp) >= config::availableTimeoutTime) {
            ESP_LOGW(TAG, "bmp timeouted");
            if (mqttClient && mqttConnected)
                mqttVerbosePub(config::topic_bmp085_availability, "offline", 0, 1);
            lastBmpValue = std::nullopt;
        }
    }
}
} // namespace deckenlampe
