#pragma once

// system includes
#include <optional>

// local includes
#include "espchrono.h"

namespace deckenlampe {
struct DhtValue
{
    espchrono::millis_clock::time_point timestamp;
    float temperature;
    float humidity;
};
extern std::optional<DhtValue> lastDhtValue;
extern espchrono::millis_clock::time_point last_dht11_temperature_pub;
extern espchrono::millis_clock::time_point last_dht11_humidity_pub;
extern bool dhtInitialized;

void init_dht();
void update_dht();
} // namespace deckenlampe
