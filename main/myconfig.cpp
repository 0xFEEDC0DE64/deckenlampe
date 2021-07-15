#include "myconfig.h"

// esp-idf includes
#include <nvs_flash.h>
#include <esp_log.h>

using namespace std::chrono_literals;

namespace deckenlampe {
nvs_handle_t nvsHandle;

namespace config {
ConfigWrapper<std::string> hostname{"hostname", "hostname", "deckenlampe1"};

ConfigWrapper<std::string> sta_ssid{"sta_ssid", "sta_ssid", "ScheissAP"};
ConfigWrapper<std::string> sta_key{"sta_key", "sta_key", "Passwort_123"};

ConfigWrapper<std::string> ap_ssid{"ap_ssid", "ap_ssid", "deckenlampe1"};
ConfigWrapper<std::string> ap_key{"ap_key", "ap_key", "Passwort_123"};

ConfigWrapper<bool> enable_webserver{"enable_webserver", "enableebserver", true};

ConfigWrapper<bool> enable_mdns{"enable_mdns", "enable_mdns", true};

ConfigWrapper<bool> enable_mqtt{"enable_mqtt", "enable_mqtt", true};
ConfigWrapper<std::string> broker_url{"broker_url", "broker_url", "mqtt://192.168.0.2/"};

ConfigWrapper<bool> enable_lamp{"enable_lamp", "enable_lamp", true};
ConfigWrapper<gpio_num_t> pins_lamp{"pins_lamp", "pins_lamp", GPIO_NUM_4};
ConfigWrapper<bool> invertLamp{"invertLamp", "invertLamp", true};
ConfigWrapper<std::string> topic_lamp_availability{"topic_lamp_availability", "topiclampavaila", "dahoam/wohnzimmer/deckenlicht1/available"};
ConfigWrapper<std::string> topic_lamp_status{"topic_lamp_status", "topic_lamp_stat", "dahoam/wohnzimmer/deckenlicht1/status"};
ConfigWrapper<std::string> topic_lamp_set{"topic_lamp_set", "topic_lamp_set", "dahoam/wohnzimmer/deckenlicht1/set"};


ConfigWrapper<bool> enable_switch{"enable_switch", "enable_switch", true};
ConfigWrapper<gpio_num_t> pins_switch{"pins_switch", "pins_switch", GPIO_NUM_35};
ConfigWrapper<bool> invert_switch{"invert_switch", "invert_switch", true};
ConfigWrapper<std::string> topic_switch_availability{"topic_switch_availability", "tpcswitchavaila", "dahoam/wohnzimmer/schalter1/available"};
ConfigWrapper<std::string> topic_switch_status{"topic_switch_status", "topicswitchstat", "dahoam/wohnzimmer/schalter1/status"};

ConfigWrapper<bool> enable_dht{"enable_dht", "enable_dht", true};
ConfigWrapper<gpio_num_t> pins_dht{"pins_dht", "pins_dht", GPIO_NUM_33};
ConfigWrapper<std::string> topic_dht11_availability{"topic_dht11_availability", "tpcdht11availab", "dahoam/wohnzimmer/dht11_1/available"};
ConfigWrapper<std::string> topic_dht11_temperature{"topic_dht11_temperature", "tpcdht11tempera", "dahoam/wohnzimmer/dht11_1/temperature"};
ConfigWrapper<std::string> topic_dht11_humidity{"topic_dht11_humidity", "tpcdht11humidit", "dahoam/wohnzimmer/dht11_1/humidity"};

ConfigWrapper<bool> enable_i2c{"enable_i2c", "enable_i2c", true};
ConfigWrapper<gpio_num_t> pins_sda{"pins_sda", "pins_sda", GPIO_NUM_16};
ConfigWrapper<gpio_num_t> pins_scl{"pins_scl", "pins_scl", GPIO_NUM_17};

ConfigWrapper<bool> enable_tsl{"enable_tsl", "enable_tsl", true};
ConfigWrapper<std::string> topic_tsl2561_availability{"topic_tsl2561_availability", "tpctsl2561avail", "dahoam/wohnzimmer/tsl2561_1/available"};
ConfigWrapper<std::string> topic_tsl2561_lux{"topic_tsl2561_lux", "tpc_tsl2561_lux", "dahoam/wohnzimmer/tsl2561_1/lux"};

ConfigWrapper<bool> enable_bmp{"enable_bmp", "enable_bmp", true};
ConfigWrapper<std::string> topic_bmp085_availability{"topic_bmp085_availability", "topicbmp085avai", "dahoam/wohnzimmer/bmp085_1/available"};
ConfigWrapper<std::string> topic_bmp085_pressure{"topic_bmp085_pressure", "tpcbmp085pressu", "dahoam/wohnzimmer/bmp085_1/pressure"};
ConfigWrapper<std::string> topic_bmp085_temperature{"topic_bmp085_temperature", "tpcbmp085temper", "dahoam/wohnzimmer/bmp085_1/temperature"};
ConfigWrapper<std::string> topic_bmp085_altitude{"topic_bmp085_altitude", "tpcbmp085altitu", "dahoam/wohnzimmer/bmp085_1/altitude"};

ConfigWrapper<espchrono::seconds32> availableTimeoutTime{"availableTimeoutTime", "availTimeouTime", 1min};
ConfigWrapper<espchrono::seconds32> valueUpdateInterval{"valueUpdateInterval", "valUpdaInterval", 15s};
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

    config::foreachConfig([](auto &config){ loadParam(config); });
}

void update_config()
{
}

namespace {
template<typename T>
void loadParam(ConfigWrapper<T> &config)
{
    if (const auto len = std::strlen(config.nvsKey()); len > 15) {
        ESP_LOGE(TAG, "%s too long %zd (%zd)", config.nvsKey(), len, len-15);
        assert(false);
    }

    if (const auto value = config.readFromFlash()) {
        if (*value)
            config.setValue(**value);
    } else
        ESP_LOGE(TAG, "error loading config %s: %.*s", config.name(), value.error().size(), value.error().data());
}
} // namespace
} // namespace deckenlamp
