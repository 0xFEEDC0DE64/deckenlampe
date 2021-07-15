#pragma once

// system includes
#include <atomic>
#include <string_view>

// local includes
#include "wrappers/mqtt_client.h"

namespace deckenlampe {
extern espcpputils::mqtt_client mqttClient;
extern std::atomic<bool> mqttStarted;
extern std::atomic<bool> mqttConnected;

void init_mqtt();
void update_mqtt();

int mqttVerbosePub(std::string_view topic, std::string_view value, int qos, int retain);
} // namespace deckenlampe
