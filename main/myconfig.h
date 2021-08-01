#pragma once

// system includes
#include <string>

// esp-idf includes
#include <nvs.h>
#include <hal/gpio_types.h>

// local includes
#include "espchrono.h"
#include "configwrapper.h"

namespace deckenlampe {
extern nvs_handle_t nvsHandle;

namespace config {
extern ConfigWrapper<std::string> hostname;

extern ConfigWrapper<std::string> sta_ssid;
extern ConfigWrapper<std::string> sta_key;

extern ConfigWrapper<std::string> ap_ssid;
extern ConfigWrapper<std::string> ap_key;

extern ConfigWrapper<bool> enable_ble;
extern ConfigWrapper<std::string> ble_name;

extern ConfigWrapper<bool> enable_webserver;

extern ConfigWrapper<bool> enable_mdns;

extern ConfigWrapper<bool> enable_mqtt;
extern ConfigWrapper<std::string> broker_url;

extern ConfigWrapper<bool> enable_lamp;
extern ConfigWrapper<gpio_num_t> pins_lamp;
extern ConfigWrapper<bool> invertLamp;
extern ConfigWrapper<std::string> topic_lamp_availability;
extern ConfigWrapper<std::string> topic_lamp_status;
extern ConfigWrapper<std::string> topic_lamp_set;


extern ConfigWrapper<bool> enable_switch;
extern ConfigWrapper<gpio_num_t> pins_switch;
extern ConfigWrapper<bool> invert_switch;
extern ConfigWrapper<std::string> topic_switch_availability;
extern ConfigWrapper<std::string> topic_switch_status;

extern ConfigWrapper<bool> enable_dht;
extern ConfigWrapper<gpio_num_t> pins_dht;
extern ConfigWrapper<bool> dht_11_or_22;
extern ConfigWrapper<std::string> topic_dht11_availability;
extern ConfigWrapper<std::string> topic_dht11_temperature;
extern ConfigWrapper<std::string> topic_dht11_humidity;

extern ConfigWrapper<bool> enable_i2c;
extern ConfigWrapper<gpio_num_t> pins_sda;
extern ConfigWrapper<gpio_num_t> pins_scl;

extern ConfigWrapper<bool> enable_tsl;
extern ConfigWrapper<std::string> topic_tsl2561_availability;
extern ConfigWrapper<std::string> topic_tsl2561_lux;

extern ConfigWrapper<bool> enable_bmp;
extern ConfigWrapper<std::string> topic_bmp085_availability;
extern ConfigWrapper<std::string> topic_bmp085_pressure;
extern ConfigWrapper<std::string> topic_bmp085_temperature;
extern ConfigWrapper<std::string> topic_bmp085_altitude;

extern ConfigWrapper<espchrono::seconds32> availableTimeoutTime;
extern ConfigWrapper<espchrono::seconds32> valueUpdateInterval;

template<typename T>
void foreachConfig(T &&callback)
{
    callback(hostname);
    callback(sta_ssid);
    callback(sta_key);
    callback(ap_ssid);
    callback(ap_key);
    callback(enable_ble);
    callback(ble_name);
    callback(enable_webserver);
    callback(enable_mdns);
    callback(enable_mqtt);
    callback(broker_url);
    callback(enable_lamp);
    callback(pins_lamp);
    callback(invertLamp);
    callback(topic_lamp_availability);
    callback(topic_lamp_status);
    callback(topic_lamp_set);
    callback(enable_switch);
    callback(pins_switch);
    callback(invert_switch);
    callback(topic_switch_availability);
    callback(topic_switch_status);
    callback(enable_dht);
    callback(pins_dht);
    callback(dht_11_or_22);
    callback(topic_dht11_availability);
    callback(topic_dht11_temperature);
    callback(topic_dht11_humidity);
    callback(enable_i2c);
    callback(pins_sda);
    callback(pins_scl);
    callback(enable_tsl);
    callback(topic_tsl2561_availability);
    callback(topic_tsl2561_lux);
    callback(enable_bmp);
    callback(topic_bmp085_availability);
    callback(topic_bmp085_pressure);
    callback(topic_bmp085_temperature);
    callback(topic_bmp085_altitude);
    callback(availableTimeoutTime);
    callback(valueUpdateInterval);
}
} // namespace config

void init_config();
void update_config();
} // namespace deckenlampe
