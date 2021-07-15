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
        .hostname = config::hostname.value(),
        .wifis = std::array<wifi_stack::wifi_entry, 10> {
            wifi_stack::wifi_entry { .ssid = config::sta_ssid.value(), .key = config::sta_key.value() },
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
                .ssid = config::ap_ssid.value(),
                .key = config::ap_key.value()
            },
            .channel = 1,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .ssid_hidden = false,
            .max_connection = 4,
            .beacon_interval = 100,
            .ip = ap_ip,
            .subnet = ap_subnet
        },
        .min_rssi = -90
    };
}
} // namespace
} // namespace deckenlampe