#include "mymdns.h"

// esp-idf includes
#include <mdns.h>
#include <esp_log.h>

// local includes
#include "myconfig.h"

namespace deckenlampe {
namespace {
constexpr const char * const TAG = "MDNS";
} // namespace

void init_mdns()
{
    if (!config::enable_mdns.value())
        return;

    {
        const auto result = mdns_init();
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "mdns_init(): %s", esp_err_to_name(result));
        if (result != ESP_OK)
            return;
    }

    {
        const auto result = mdns_hostname_set(config::hostname.value().c_str());
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "mdns_hostname_set(): %s", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    {
        const auto result = mdns_instance_name_set(config::hostname.value().c_str());
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "mdns_instance_name_set(): %s", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }

    if (config::enable_webserver.value())
    {
        const auto result = mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "mdns_service_add(): %s", esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }
}

void update_mdns()
{
    if (!config::enable_mdns.value())
        return;
}
} // namespace deckenlampe
