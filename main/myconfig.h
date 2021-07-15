#pragma once

// system includes
#include <string_view>
#include <chrono>

// esp-idf includes
#include <nvs.h>
#include <hal/gpio_types.h>

namespace deckenlampe {
extern nvs_handle_t nvsHandle;

namespace config {
constexpr const std::string_view hostname = "deckenlampe1";

constexpr const std::string_view sta_ssid = "ScheissAP";
constexpr const std::string_view sta_key = "Passwort_123";

constexpr const std::string_view ap_ssid = "deckenlampe1";
constexpr const std::string_view ap_key = "Passwort_123";

constexpr const bool enable_webserver = true;

constexpr const bool enable_mdns = true;

constexpr const bool enable_mqtt = true;
constexpr const std::string_view broker_url = "mqtt://192.168.0.2/";

constexpr const bool enable_lamp = true;
constexpr const gpio_num_t pins_lamp = GPIO_NUM_25;
constexpr const bool invertLamp = true;
constexpr const std::string_view topic_lamp_availability = "dahoam/wohnzimmer/deckenlicht1/available";
constexpr const std::string_view topic_lamp_status = "dahoam/wohnzimmer/deckenlicht1/status";
constexpr const std::string_view topic_lamp_set = "dahoam/wohnzimmer/deckenlicht1/set";


constexpr const bool enable_switch = true;
constexpr const gpio_num_t pins_switch = GPIO_NUM_35;
constexpr const bool invert_switch = true;
constexpr const std::string_view topic_switch_availability = "dahoam/wohnzimmer/schalter1/available";
constexpr const std::string_view topic_switch_status = "dahoam/wohnzimmer/schalter1/status";

constexpr const bool enable_dht = true;
constexpr const gpio_num_t pins_dht = GPIO_NUM_33;
constexpr const std::string_view topic_dht11_availability = "dahoam/wohnzimmer/dht11_1/available";
constexpr const std::string_view topic_dht11_temperature = "dahoam/wohnzimmer/dht11_1/temperature";
constexpr const std::string_view topic_dht11_humidity = "dahoam/wohnzimmer/dht11_1/humidity";

constexpr const bool enable_i2c = true;
constexpr const gpio_num_t pins_sda = GPIO_NUM_16;
constexpr const gpio_num_t pins_scl = GPIO_NUM_17;

constexpr const bool enable_tsl = true;
constexpr const std::string_view topic_tsl2561_availability = "dahoam/wohnzimmer/tsl2561_1/available";
constexpr const std::string_view topic_tsl2561_lux = "dahoam/wohnzimmer/tsl2561_1/lux";

constexpr const bool enable_bmp = true;
constexpr const std::string_view topic_bmp085_availability = "dahoam/wohnzimmer/bmp085_1/available";
constexpr const std::string_view topic_bmp085_pressure = "dahoam/wohnzimmer/bmp085_1/pressure";
constexpr const std::string_view topic_bmp085_temperature = "dahoam/wohnzimmer/bmp085_1/temperature";
constexpr const std::string_view topic_bmp085_altitude = "dahoam/wohnzimmer/bmp085_1/altitude";

constexpr const std::chrono::minutes availableTimeoutTime{1};
constexpr const std::chrono::seconds valueUpdateInterval{15};
} // namespace configs

void init_config();
void update_config();
} // namespace deckenlampe
