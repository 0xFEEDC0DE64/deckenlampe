#include "myota.h"

// esp-idf includes
#include <esp_log.h>

// local includes
#include "delayedconstruction.h"
#include "espasyncota.h"
#include "espwifistack.h"

namespace deckenlampe {
namespace {
constexpr const char * const TAG = "OTA";

cpputils::DelayedConstruction<EspAsyncOta> _asyncOta;
} // namespace

EspAsyncOta &asyncOta{_asyncOta.getUnsafe()};

void init_ota()
{
    ESP_LOGI(TAG, "called");

    _asyncOta.construct();

    if (const auto result = _asyncOta->startTask(); !result)
    {
        ESP_LOGE(TAG, "starting OTA task failed: %.*s", result.error().size(), result.error().data());
        return;
    }
}

void update_ota()
{
    _asyncOta->update();
}

tl::expected<void, std::string> triggerOta(std::string_view url)
{
    ESP_LOGI(TAG, "%.*s", url.size(), url.data());

    if (const auto result = _asyncOta->trigger(url, {}, {}, {}); !result)
        return tl::make_unexpected(std::move(result).error());

    wifi_stack::delete_scan_result();

    return {};
}
} // namespace deckenlampe
