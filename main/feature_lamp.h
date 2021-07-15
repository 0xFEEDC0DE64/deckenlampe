#pragma once

// system includes
#include <atomic>

namespace deckenlampe {
extern std::atomic<bool> lampState;

void init_lamp();
void update_lamp();

void writeLamp(bool state);
} // namespace deckenlampe
