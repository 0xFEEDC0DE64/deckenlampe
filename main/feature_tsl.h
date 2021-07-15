#pragma once

// system includes
#include <optional>

// local includes
#include "espchrono.h"

namespace deckenlampe {
struct TslValue
{
    espchrono::millis_clock::time_point timestamp;
    float lux;
};
extern std::optional<TslValue> lastTslValue;
extern espchrono::millis_clock::time_point last_tsl2561_lux_pub;
extern bool tslInitialized;

void init_tsl();
void update_tsl();
} // namespace deckenlampe
