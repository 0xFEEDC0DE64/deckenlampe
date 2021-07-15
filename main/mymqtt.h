#pragma once

// system includes
#include <atomic>
#include <string_view>

// local includes
#include "wrappers/mqtt_client.h"

namespace deckenlampe {
extern std::atomic<bool> mqttConnected;
extern espcpputils::mqtt_client mqttClient;

void init_mqtt();
void update_mqtt();

int mqttVerbosePub(std::string_view topic, std::string_view value, int qos, int retain);
} // namespace deckenlampe
