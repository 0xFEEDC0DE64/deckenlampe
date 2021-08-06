#pragma once

// system includes
#include <string>
#include <string_view>

// 3rdparty lib includes
#include <tl/expected.hpp>
// forward declares
class EspAsyncOta;

namespace deckenlampe {
extern EspAsyncOta &asyncOta;

void init_ota();
void update_ota();

tl::expected<void, std::string> triggerOta(std::string_view url);
} // namespace deckenlampe
