#include "sdkconfig.h"

// system includes
#include <chrono>

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
#include <nvs_flash.h>
#include <esp_http_server.h>

// Arduino includes
#include <Arduino.h>

// local includes
#include "espwifistack.h"
#include "tickchrono.h"

using namespace std::chrono_literals;

namespace {
constexpr const char * const TAG = "MAIN";
} // namespace

extern "C" void app_main()
{
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

    {
        const auto result = nvs_flash_init();
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "nvs_flash_init(): %s", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    nvs_handle_t nvsHandle;

    {
        const auto result = nvs_open("deckenlampe", NVS_READWRITE, &nvsHandle);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "nvs_open(): %s", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    wifi_stack::config wifiConfig {
        .wifiEnabled = true,
        .hostname = "deckenlampe1",
        .wifis = std::array<wifi_stack::wifi_entry, 10> {
            wifi_stack::wifi_entry { .ssid = "McDonalds Free WiFi", .key = "Passwort_123" },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} },
            wifi_stack::wifi_entry { .ssid = {}, .key = {} }
        },
        .sta_ip = {
            .dhcpEnabled = true,
//            .staticIp = {},
//            .staticGateway = {},
//            .staticSubnet = {},
//            .staticDns1 = {},
//            .staticDns2 = {}
        },
        .ap = {
            {
                .ssid = "deckenlampe1",
                .key = "Passwort_123"
            },
            .channel = 1,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .ssid_hidden = false,
            .max_connection = 4,
            .beacon_interval = 100,
            .ip{192, 168, 4, 1},
            .subnet{255, 255, 255, 0}
        },
        .min_rssi = -90
    };

    wifi_stack::init(wifiConfig);

    httpd_handle_t httpdHandle{};

    {
        httpd_config_t httpConfig HTTPD_DEFAULT_CONFIG();

        const auto result = httpd_start(&httpdHandle, &httpConfig);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "httpd_start(): %s", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    while (true)
    {
        wifi_stack::update(wifiConfig);

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

        vTaskDelay(std::chrono::ceil<espcpputils::ticks>(100ms).count());
    }
}
