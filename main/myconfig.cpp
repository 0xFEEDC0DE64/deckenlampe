#include "myconfig.h"

// esp-idf includes
#include <nvs_flash.h>
#include <esp_log.h>

namespace deckenlampe {
nvs_handle_t nvsHandle;

namespace {
constexpr const char * const TAG = "CONFIG";
} // namespace

void init_config()
{
    {
        const auto result = nvs_flash_init();
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "nvs_flash_init(): %s", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    {
        const auto result = nvs_open("deckenlampe", NVS_READWRITE, &nvsHandle);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "nvs_open(): %s", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    // TODO
}

void update_config()
{
}
} // namespace deckenlamp
