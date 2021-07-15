#include "sdkconfig.h"

// esp-idf includes
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#if defined(CONFIG_ESP_TASK_WDT_PANIC) || defined(CONFIG_ESP_TASK_WDT)
#include <freertos/task.h>
#include <esp_task_wdt.h>
#endif
#ifdef CONFIG_APP_ROLLBACK_ENABLE
#include <esp_ota_ops.h>
#endif
#include <esp_system.h>

// Arduino includes
#include <Wire.h>

// local includes
#include "tickchrono.h"
#include "myconfig.h"
#include "mywifi.h"
#include "webserver.h"
#include "mymdns.h"
#include "mymqtt.h"
#include "feature_lamp.h"
#include "feature_switch.h"
#include "feature_dht.h"
#include "feature_tsl.h"
#include "feature_bmp.h"
#include "espchrono.h"

using namespace std::chrono_literals;

namespace deckenlampe {
namespace {
constexpr const char * const TAG = "MAIN";

} // namespace
} // namespace deckenlamp

extern "C" void app_main()
{
    using namespace deckenlampe;

#if defined(CONFIG_ESP_TASK_WDT_PANIC) || defined(CONFIG_ESP_TASK_WDT)
    {
        const auto taskHandle = xTaskGetCurrentTaskHandle();
        if (!taskHandle)
        {
            ESP_LOGE(TAG, "could not get handle to current main task!");
        }
        else if (const auto result = esp_task_wdt_add(taskHandle); result != ESP_OK)
        {
            ESP_LOGE(TAG, "could not add main task to watchdog: %s", esp_err_to_name(result));
        }
    }
#endif

#ifdef CONFIG_APP_ROLLBACK_ENABLE
    esp_ota_img_states_t ota_state;
    if (const esp_partition_t * const running = esp_ota_get_running_partition())
    {
        if (const auto result = esp_ota_get_state_partition(running, &ota_state); result == ESP_ERR_NOT_FOUND)
            ota_state = ESP_OTA_IMG_VALID;
        else if (result != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_ota_get_state_partition() failed with %s", esp_err_to_name(result));
            ota_state = ESP_OTA_IMG_UNDEFINED;
        }
    }
    else
    {
        ESP_LOGE(TAG, "esp_ota_get_running_partition() returned nullptr");
        ota_state = ESP_OTA_IMG_UNDEFINED;
    }
#endif

    init_config();

    init_lamp();

    init_switch();

    if (config::enable_i2c)
    {
        ESP_LOGI(TAG, "calling Wire.begin()...");
        const auto result = Wire.begin(config::pins_sda, config::pins_scl);
        ESP_LOGI(TAG, "finished with %s", result ? "true" : "false");
    }

    init_wifi();

    init_webserver();

    init_mdns();

    init_mqtt();

    init_dht();

    init_tsl();

    init_bmp();

    while (true)
    {
        update_config();

        update_lamp();

        update_wifi();

        update_webserver();

        update_mdns();

        update_mqtt();

        update_dht();

        update_tsl();

        update_bmp();

        update_switch();

#if defined(CONFIG_ESP_TASK_WDT_PANIC) || defined(CONFIG_ESP_TASK_WDT)
        if (const auto result = esp_task_wdt_reset(); result != ESP_OK)
            ESP_LOGE(TAG, "esp_task_wdt_reset() failed with %s", esp_err_to_name(result));
#endif

#ifdef CONFIG_APP_ROLLBACK_ENABLE
        if (ota_state == ESP_OTA_IMG_PENDING_VERIFY)
        {
            const auto result = esp_ota_mark_app_valid_cancel_rollback();
            ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "esp_ota_mark_app_valid_cancel_rollback() returned: %s", esp_err_to_name(result));
            ota_state = ESP_OTA_IMG_VALID;
        }
#endif

        vPortYield();
    }
}

auto espchrono::local_clock::timezone() noexcept -> time_zone
{
    return time_zone{1h, DayLightSavingMode::EuropeanSummerTime};
}
