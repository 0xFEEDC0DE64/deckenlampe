#include "feature_switch.h"

// Arduino includes
#include <Arduino.h>

// local includes
#include "myconfig.h"
#include "mymqtt.h"
#include "feature_lamp.h"
#include "espchrono.h"

using namespace std::chrono_literals;

namespace deckenlampe {
std::atomic<bool> switchState;

namespace {
uint8_t switchDebounce{};
espchrono::millis_clock::time_point last_switch_readout;

bool readSwitch();
} // namespace

void init_switch()
{
    if (!config::enable_switch)
        return;

    pinMode(config::pins_switch, INPUT);
}

void update_switch()
{
    if (!config::enable_switch)
        return;

    if (espchrono::ago(last_switch_readout) < 20ms)
        return;
    last_switch_readout = espchrono::millis_clock::now();

    const auto newState = readSwitch();
    if (newState == switchState.load())
        switchDebounce = 0;
    else {
        switchDebounce++;
        if (switchDebounce >= 10) {
            switchDebounce = 0;

            switchState = newState;

            if (mqttConnected)
                mqttVerbosePub(config::topic_switch_status, switchState ? "ON" : "OFF", 0, 1);

            if (config::enable_lamp) {
                lampState = !lampState;
                writeLamp(lampState);

                if (mqttConnected)
                    mqttVerbosePub(config::topic_lamp_status, lampState ? "ON" : "OFF", 0, 1);
            }
        }
    }
}

namespace {
bool readSwitch()
{
    bool state = digitalRead(config::pins_switch) == HIGH;
    if (config::invert_switch)
        state = !state;
    return state;
}
} // namespace
} // namespace deckenlampe
