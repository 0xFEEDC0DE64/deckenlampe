#pragma once

// local includes
#include "espwifiutils.h"

namespace deckenlampe {
constexpr const wifi_stack::ip_address_t ap_ip{192, 168, 4, 1};
constexpr const wifi_stack::ip_address_t ap_subnet{255, 255, 255, 0};

void init_wifi();
void update_wifi();
} // namespace deckenlampe
