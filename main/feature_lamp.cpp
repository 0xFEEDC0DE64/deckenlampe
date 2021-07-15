#include "feature_lamp.h"

// Arduino includes
#include <Arduino.h>

// local includes
#include <myconfig.h>

namespace deckenlampe {
std::atomic<bool> lampState;

void init_lamp()
{
    if (!config::enable_lamp.value())
        return;

    pinMode(config::pins_lamp.value(), OUTPUT);
    writeLamp(lampState);
}

void update_lamp()
{
    if (!config::enable_lamp.value())
        return;
}

void writeLamp(bool state)
{
    if (config::invertLamp.value())
        state = !state;
    digitalWrite(config::pins_lamp.value(), state ? HIGH : LOW);
}
} // namespace deckenlampe
