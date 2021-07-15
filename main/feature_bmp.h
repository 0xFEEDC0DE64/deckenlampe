#pragma once

// system includes
#include <optional>

// local includes
#include "espchrono.h"

namespace deckenlampe {
struct BmpValue
{
    espchrono::millis_clock::time_point timestamp;
    float pressure;
    float temperature;
    float altitude;
};
extern std::optional<BmpValue> lastBmpValue;
extern espchrono::millis_clock::time_point last_bmp085_pressure_pub;
extern espchrono::millis_clock::time_point last_bmp085_temperature_pub;
extern espchrono::millis_clock::time_point last_bmp085_altitude_pub;
extern bool bmpInitialized;

void init_bmp();
void update_bmp();
} // namespace deckenlampe
