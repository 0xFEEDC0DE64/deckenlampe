#include "feature_dht.h"

// esp-idf includes
#include <esp_log.h>

// 3rdparty lib includes
#include <fmt/core.h>
#include <DHT.h>

// local includes
#include "myconfig.h"
#include "mymqtt.h"
#include "delayedconstruction.h"

using namespace std::chrono_literals;

namespace deckenlampe {
std::optional<DhtValue> lastDhtValue;
espchrono::millis_clock::time_point last_dht11_temperature_pub;
espchrono::millis_clock::time_point last_dht11_humidity_pub;
bool dhtInitialized{};

namespace {
constexpr const char * const TAG = "DHT";

cpputils::DelayedConstruction<DHT> dht;
espchrono::millis_clock::time_point last_dht11_readout;
} // namespace

void init_dht()
{
    if (!config::enable_dht)
        return;

    dht.construct(config::pins_dht, DHT11);

    ESP_LOGI(TAG, "calling dht.begin()...");
    dhtInitialized = dht->begin();
    ESP_LOGI(TAG, "finished with %s", dhtInitialized ? "true" : "false");
}

void update_dht()
{
    if (!config::enable_dht)
        return;

    if (dhtInitialized) {
        if (espchrono::ago(last_dht11_readout) < 5s)
            return;

        last_dht11_readout = espchrono::millis_clock::now();

        if (const auto data = dht->read()) {
            DhtValue dhtValue {
                .timestamp = espchrono::millis_clock::now(),
                .temperature = dht->readTemperature(*data),
                .humidity = dht->readHumidity(*data)
            };

            ESP_LOGI(TAG, "read dht temperature: %.1f C", dhtValue.temperature);
            ESP_LOGI(TAG, "read dht humidity: %.1f %%", dhtValue.humidity);

            if (mqttConnected) {
                if (!lastDhtValue)
                    mqttVerbosePub(config::topic_dht11_availability, "online", 0, 1);
                if (!lastDhtValue || espchrono::ago(last_dht11_temperature_pub) >= config::valueUpdateInterval) {
                    if (mqttVerbosePub(config::topic_dht11_temperature, fmt::format("{:.1f}", dhtValue.temperature), 0, 1) >= 0)
                        last_dht11_temperature_pub = espchrono::millis_clock::now();
                }
                if (!lastDhtValue || espchrono::ago(last_dht11_humidity_pub) >= config::valueUpdateInterval) {
                    if (mqttVerbosePub(config::topic_dht11_humidity, fmt::format("{:.1f}", dhtValue.humidity), 0, 1) >= 0)
                        last_dht11_humidity_pub = espchrono::millis_clock::now();
                }
            }

            lastDhtValue = dhtValue;
        } else {
            ESP_LOGW(TAG, "dht failed");
            goto dhtOffline;
        }
    } else {
    dhtOffline:
        if (lastDhtValue && espchrono::ago(lastDhtValue->timestamp) >= config::availableTimeoutTime) {
            ESP_LOGW(TAG, "dht timeouted");
            if (mqttConnected)
                mqttVerbosePub(config::topic_dht11_availability, "offline", 0, 1);
            lastDhtValue = std::nullopt;
        }
    }
}
} // namespace deckenlampe
