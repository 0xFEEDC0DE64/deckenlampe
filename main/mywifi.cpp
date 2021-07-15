#include "mywifi.h"

// local includes
#include "espwifistack.h"
#include "myconfig.h"

namespace deckenlampe {
namespace {
wifi_stack::config makeWifiConfig();
} // namespace

void init_wifi()
{
    wifi_stack::init(makeWifiConfig());
}

void update_wifi()
{
    wifi_stack::update(makeWifiConfig());
}

namespace {
wifi_stack::config makeWifiConfig()
{
    return wifi_stack::config {
        .wifiEnabled = true,
        .hostname = std::string{config::hostname},
        .wifis = std::array<wifi_stack::wifi_entry, 10> {
            wifi_stack::wifi_entry { .ssid = std::string{config::sta_ssid}, .key = std::string{config::sta_key} },
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
                .ssid = std::string{config::ap_ssid},
                .key = std::string{config::ap_key}
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
}
} // namespace
} // namespace deckenlampe
