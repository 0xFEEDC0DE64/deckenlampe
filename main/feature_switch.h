#pragma once

// system includes
#include <atomic>

namespace deckenlampe {
extern std::atomic<bool> switchState;

void init_switch();
void update_switch();
} // namespace deckenlampe
