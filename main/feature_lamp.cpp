#include "feature_lamp.h"

// Arduino includes
#include <Arduino.h>

// local includes
#include <myconfig.h>

namespace deckenlampe {
std::atomic<bool> lampState;

void init_lamp()
{
    if (!config::enable_lamp)
        return;

    pinMode(config::pins_lamp, OUTPUT);
    writeLamp(lampState);
}

void update_lamp()
{
    if (!config::enable_lamp)
        return;
}

void writeLamp(bool state)
{
    if (config::invertLamp)
        state = !state;
    digitalWrite(config::pins_lamp, state ? HIGH : LOW);
}
} // namespace deckenlampe
