#include "myconfig.h"

// esp-idf includes
#include <nvs_flash.h>
#include <esp_log.h>

using namespace std::chrono_literals;

namespace deckenlampe {
nvs_handle_t nvsHandle;

namespace config {
ConfigWrapper<std::string> hostname{"hostname", "deckenlampe1"};

ConfigWrapper<std::string> sta_ssid{"sta_ssid", "ScheissAP"};
ConfigWrapper<std::string> sta_key{"sta_key", "Passwort_123"};

ConfigWrapper<std::string> ap_ssid{"ap_ssid", "deckenlampe1"};
ConfigWrapper<std::string> ap_key{"ap_key", "Passwort_123"};

ConfigWrapper<bool> enable_webserver{"enable_webserver", true};

ConfigWrapper<bool> enable_mdns{"enable_mdns", true};

ConfigWrapper<bool> enable_mqtt{"enable_mqtt", true};
ConfigWrapper<std::string> broker_url{"broker_url", "mqtt://192.168.0.2/"};

ConfigWrapper<bool> enable_lamp{"enable_lamp", true};
ConfigWrapper<gpio_num_t> pins_lamp{"pins_lamp", GPIO_NUM_4};
ConfigWrapper<bool> invertLamp{"invertLamp", true};
ConfigWrapper<std::string> topic_lamp_availability{"topic_lamp_availability", "dahoam/wohnzimmer/deckenlicht1/available"};
ConfigWrapper<std::string> topic_lamp_status{"topic_lamp_status", "dahoam/wohnzimmer/deckenlicht1/status"};
ConfigWrapper<std::string> topic_lamp_set{"topic_lamp_set", "dahoam/wohnzimmer/deckenlicht1/set"};


ConfigWrapper<bool> enable_switch{"enable_switch", true};
ConfigWrapper<gpio_num_t> pins_switch{"pins_switch", GPIO_NUM_35};
ConfigWrapper<bool> invert_switch{"invert_switch", true};
ConfigWrapper<std::string> topic_switch_availability{"topic_switch_availability", "dahoam/wohnzimmer/schalter1/available"};
ConfigWrapper<std::string> topic_switch_status{"topic_switch_status", "dahoam/wohnzimmer/schalter1/status"};

ConfigWrapper<bool> enable_dht{"enable_dht", true};
ConfigWrapper<gpio_num_t> pins_dht{"pins_dht", GPIO_NUM_33};
ConfigWrapper<std::string> topic_dht11_availability{"topic_dht11_availability", "dahoam/wohnzimmer/dht11_1/available"};
ConfigWrapper<std::string> topic_dht11_temperature{"topic_dht11_temperature", "dahoam/wohnzimmer/dht11_1/temperature"};
ConfigWrapper<std::string> topic_dht11_humidity{"topic_dht11_humidity", "dahoam/wohnzimmer/dht11_1/humidity"};

ConfigWrapper<bool> enable_i2c{"enable_i2c", true};
ConfigWrapper<gpio_num_t> pins_sda{"pins_sda", GPIO_NUM_16};
ConfigWrapper<gpio_num_t> pins_scl{"pins_scl", GPIO_NUM_17};

ConfigWrapper<bool> enable_tsl{"enable_tsl", true};
ConfigWrapper<std::string> topic_tsl2561_availability{"topic_tsl2561_availability", "dahoam/wohnzimmer/tsl2561_1/available"};
ConfigWrapper<std::string> topic_tsl2561_lux{"topic_tsl2561_lux", "dahoam/wohnzimmer/tsl2561_1/lux"};

ConfigWrapper<bool> enable_bmp{"enable_bmp", true};
ConfigWrapper<std::string> topic_bmp085_availability{"topic_bmp085_availability", "dahoam/wohnzimmer/bmp085_1/available"};
ConfigWrapper<std::string> topic_bmp085_pressure{"topic_bmp085_pressure", "dahoam/wohnzimmer/bmp085_1/pressure"};
ConfigWrapper<std::string> topic_bmp085_temperature{"topic_bmp085_temperature", "dahoam/wohnzimmer/bmp085_1/temperature"};
ConfigWrapper<std::string> topic_bmp085_altitude{"topic_bmp085_altitude", "dahoam/wohnzimmer/bmp085_1/altitude"};

ConfigWrapper<espchrono::seconds32> availableTimeoutTime{"availableTimeoutTime", 1min};
ConfigWrapper<espchrono::seconds32> valueUpdateInterval{"valueUpdateInterval", 15s};
} // namespace config

namespace {
constexpr const char * const TAG = "CONFIG";

template<typename T>
void loadParam(ConfigWrapper<T> &config);
} // namespace

void init_config()
{
    {
        const auto result = nvs_flash_init();
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "nvs_flash_init(): %s", esp_err_to_name(result));
        if (result != ESP_OK)
            return;
    }

    {
        constexpr const char* name = "deckenlampe";
        const auto result = nvs_open(name, NVS_READWRITE, &nvsHandle);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "nvs_open() %s: %s", name, esp_err_to_name(result));
        if (result != ESP_OK)
            return;
    }

    loadParam(config::hostname);
    loadParam(config::sta_ssid);
    loadParam(config::sta_key);
    loadParam(config::ap_ssid);
    loadParam(config::ap_key);
    loadParam(config::enable_webserver);
    loadParam(config::enable_mdns);
    loadParam(config::enable_mqtt);
    loadParam(config::broker_url);
    loadParam(config::enable_lamp);
    loadParam(config::pins_lamp);
    loadParam(config::invertLamp);
    loadParam(config::topic_lamp_availability);
    loadParam(config::topic_lamp_status);
    loadParam(config::topic_lamp_set);
    loadParam(config::enable_switch);
    loadParam(config::pins_switch);
    loadParam(config::invert_switch);
    loadParam(config::topic_switch_availability);
    loadParam(config::topic_switch_status);
    loadParam(config::enable_dht);
    loadParam(config::pins_dht);
    loadParam(config::topic_dht11_availability);
    loadParam(config::topic_dht11_temperature);
    loadParam(config::topic_dht11_humidity);
    loadParam(config::enable_i2c);
    loadParam(config::pins_sda);
    loadParam(config::pins_scl);
    loadParam(config::enable_tsl);
    loadParam(config::topic_tsl2561_availability);
    loadParam(config::topic_tsl2561_lux);
    loadParam(config::enable_bmp);
    loadParam(config::topic_bmp085_availability);
    loadParam(config::topic_bmp085_pressure);
    loadParam(config::topic_bmp085_temperature);
    loadParam(config::topic_bmp085_altitude);
    loadParam(config::availableTimeoutTime);
    loadParam(config::valueUpdateInterval);
}

void update_config()
{
}

namespace {
template<typename T>
void loadParam(ConfigWrapper<T> &config)
{
    if (const auto value = config.readFromFlash())
        config.setValue(*value);
    else
        ESP_LOGE(TAG, "error loading param %s: %.*s", config.key(), value.error().size(), value.error().data());
}
} // namespace
} // namespace deckenlamp
