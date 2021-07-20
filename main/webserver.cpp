#include "webserver.h"

// system includes
#include <string_view>
#include <string>
#include <atomic>

// esp-idf includes
#include <esp_log.h>

// 3rdparty lib includes
#include <fmt/core.h>

// local includes
#include "myconfig.h"
#include "feature_lamp.h"
#include "feature_switch.h"
#include "feature_dht.h"
#include "feature_tsl.h"
#include "feature_bmp.h"
#include "mymqtt.h"
#include "espwifistack.h"
#include "espcppmacros.h"
#include "espstrutils.h"
#include "strutils.h"
#include "espchrono.h"
#include "numberparsing.h"

namespace deckenlampe {
httpd_handle_t httpdHandle;

namespace {
constexpr const char * const TAG = "WEBSERVER";

template<typename T> T htmlentities(const T &val) { return val; } // TODO
template<typename T> T htmlentities(T &&val) { return val; } // TODO

std::atomic<bool> shouldReboot;

esp_err_t webserver_root_handler(httpd_req_t *req);
esp_err_t webserver_on_handler(httpd_req_t *req);
esp_err_t webserver_off_handler(httpd_req_t *req);
esp_err_t webserver_toggle_handler(httpd_req_t *req);
esp_err_t webserver_reboot_handler(httpd_req_t *req);
} // namespace

void init_webserver()
{
    if (!config::enable_webserver.value())
        return;

    {
        httpd_config_t httpConfig HTTPD_DEFAULT_CONFIG();
        httpConfig.core_id = 1;

        const auto result = httpd_start(&httpdHandle, &httpConfig);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "httpd_start(): %s", esp_err_to_name(result));
        if (result != ESP_OK)
            return;
    }

    for (const httpd_uri_t &uri : {
             httpd_uri_t { .uri = "/",       .method = HTTP_GET, .handler = webserver_root_handler,   .user_ctx = NULL },
             httpd_uri_t { .uri = "/on",     .method = HTTP_GET, .handler = webserver_on_handler,     .user_ctx = NULL },
             httpd_uri_t { .uri = "/off",    .method = HTTP_GET, .handler = webserver_off_handler,    .user_ctx = NULL },
             httpd_uri_t { .uri = "/toggle", .method = HTTP_GET, .handler = webserver_toggle_handler, .user_ctx = NULL },
             httpd_uri_t { .uri = "/reboot", .method = HTTP_GET, .handler = webserver_reboot_handler, .user_ctx = NULL }
    })
    {
        const auto result = httpd_register_uri_handler(httpdHandle, &uri);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "httpd_register_uri_handler() for %s: %s", uri.uri, esp_err_to_name(result));
        //if (result != ESP_OK)
        //    return result;
    }
}

void update_webserver()
{
    if (!config::enable_webserver.value())
        return;

    if (shouldReboot)
    {
        shouldReboot = false;
        esp_restart();
    }
}

namespace {
esp_err_t webserver_dht_display(httpd_req_t *req, std::string &body);
esp_err_t webserver_tsl_display(httpd_req_t *req, std::string &body);
esp_err_t webserver_bmp_display(httpd_req_t *req, std::string &body);
esp_err_t webserver_wifi_display(httpd_req_t *req, std::string &body);
esp_err_t webserver_mqtt_display(httpd_req_t *req, std::string &body);
esp_err_t webserver_config_display(httpd_req_t *req, std::string &body, std::string_view query);

esp_err_t webserver_root_handler(httpd_req_t *req)
{
    std::string query;

    if (const size_t queryLength = httpd_req_get_url_query_len(req)) {
        query.resize(queryLength);
        CALL_AND_EXIT_ON_ERROR(httpd_req_get_url_query_str, req, query.data(), query.size() + 1)
    }

    std::string body;

    if (config::enable_lamp.value()) {
        body += "<a href=\"/on\">on</a><br/>\n"
                "<a href=\"/off\">off</a><br/>\n"
                "<a href=\"/toggle\">toggle</a><br/>\n";
    }

    body += "<a href=\"/reboot\">reboot</a><br/>\n"
            "<br/>\n";

    if (config::enable_lamp.value())
        body += fmt::format("Lamp: {}<br/>\n", lampState ? "ON" : "OFF");

    if (config::enable_switch.value())
        body += fmt::format("Switch: {}<br/>\n", switchState ? "ON" : "OFF");

    if (const auto result = webserver_dht_display(req, body); result != ESP_OK)
        return result;

    if (const auto result = webserver_tsl_display(req, body); result != ESP_OK)
        return result;

    if (const auto result = webserver_bmp_display(req, body); result != ESP_OK)
        return result;

    body += "<br/>\n";

    if (const auto result = webserver_wifi_display(req, body); result != ESP_OK)
        return result;

    if (const auto result = webserver_mqtt_display(req, body); result != ESP_OK)
        return result;

    if (const auto result = webserver_config_display(req, body, query); result != ESP_OK)
        return result;

    CALL_AND_EXIT_ON_ERROR(httpd_resp_set_type, req, "text/html")
    CALL_AND_EXIT(httpd_resp_send, req, body.data(), body.size())
}

esp_err_t webserver_dht_display(httpd_req_t *req, std::string &body)
{
    if (config::enable_dht.value()) {
        if (lastDhtValue) {
            body += fmt::format("DHT11 Temperature: {:.1f} C<br/>\n", lastDhtValue->temperature);
            body += fmt::format("DHT11 Humidity: {:.1f} %<br/>\n", lastDhtValue->humidity);
        } else
            body += "DHT11 not available at the moment<br/>\n";
    }

    return ESP_OK;
}

esp_err_t webserver_tsl_display(httpd_req_t *req, std::string &body)
{
    if (config::enable_i2c.value() && config::enable_tsl.value()) {
        if (lastTslValue) {
            body += fmt::format("TSL2561 Brightness: {:.1f} lux<br/>\n", lastTslValue->lux);
        } else
            body += "TSL2561 not available at the moment<br/>\n";
    }

    return ESP_OK;
}

esp_err_t webserver_bmp_display(httpd_req_t *req, std::string &body)
{
    if (config::enable_i2c.value() && config::enable_bmp.value()) {
        if (lastBmpValue) {
            body += fmt::format("BMP085 Pressure: {:.1f} lux<br/>\n", lastBmpValue->pressure);
            body += fmt::format("BMP085 Temperature: {:.1f} C<br/>\n", lastBmpValue->temperature);
            body += fmt::format("BMP085 Altitude: {:.1f} m<br/>\n", lastBmpValue->altitude);
        } else
            body += "BMP085 not available at the moment<br/>\n";
    }

    return ESP_OK;
}

esp_err_t webserver_wifi_display(httpd_req_t *req, std::string &body)
{
    body += "<h2>wifi</h2>\n"
            "<table border=\"1\">\n";
    body += fmt::format("<tr><th>state machine state</th><td>{}</td></tr>\n", wifi_stack::toString(wifi_stack::wifiStateMachineState));

    {
        const auto staStatus = wifi_stack::get_sta_status();
        body += fmt::format("<tr><th>STA status</th><td>{}</td></tr>\n", wifi_stack::toString(staStatus));
        if (staStatus == wifi_stack::WiFiStaStatus::WL_CONNECTED) {
            if (const auto result = wifi_stack::get_sta_ap_info(); result) {
                body += fmt::format("<tr><th>STA rssi</th><td>{}dB</td></tr>\n", result->rssi);
                body += fmt::format("<tr><th>STA SSID</th><td>{}</td></tr>\n", htmlentities(result->ssid));
                body += fmt::format("<tr><th>STA channel</th><td>{}</td></tr>\n", result->primary);
                body += fmt::format("<tr><th>STA BSSID</th><td>{}</td></tr>\n", wifi_stack::toString(wifi_stack::mac_t{result->bssid}));
            } else {
                body += fmt::format("<tr><td colspan=\"2\">get_sta_ap_info() failed: {}</td></tr>\n", htmlentities(result.error()));
            }

            if (const auto result = wifi_stack::get_ip_info(TCPIP_ADAPTER_IF_STA)) {
                body += fmt::format("<tr><th>STA ip</th><td>{}</td></tr>\n", wifi_stack::toString(result->ip));
                body += fmt::format("<tr><th>STA netmask</th><td>{}</td></tr>\n", wifi_stack::toString(result->netmask));
                body += fmt::format("<tr><th>STA gw</th><td>{}</td></tr>\n", wifi_stack::toString(result->gw));
            } else {
                body += fmt::format("<tr><td colspan=\"2\">get_ip_info() failed: {}</td></tr>\n", htmlentities(result.error()));
            }
        }
    }

    body += fmt::format("<tr><th>scan status</th><td>{}</td></tr>\n", wifi_stack::toString(wifi_stack::get_scan_status()));
    body += "</table>\n"
            "<h3>scan result</h3>\n";

    if (const auto &scanResult = wifi_stack::get_scan_result()) {
        body += fmt::format("scan age: {}s<br />\n", std::chrono::round<std::chrono::seconds>(espchrono::ago(scanResult->finished)).count());
        body += "<table border=\"1\">\n"
                    "<thead>\n"
                        "<tr>\n"
                            "<th>ssid</th>\n"
                            "<th>authmode</th>\n"
                            "<th>pairwise_cipher</th>\n"
                            "<th>group_cipher</th>\n"
                            "<th>rssi</th>\n"
                            "<th>channel</th>\n"
                            "<th>bssid</th>\n"
                        "</tr>\n"
                    "</thead>\n"
                    "<tbody>\n";

        for (const auto &entry : scanResult->entries) {
            body += fmt::format(
                "<tr>\n"
                    "<td>{}</td>\n"
                    "<td>{}</td>\n"
                    "<td>{}</td>\n"
                    "<td>{}</td>\n"
                    "<td>{}</td>\n"
                    "<td>{}</td>\n"
                    "<td>{}</td>\n"
                "</tr>\n",
                htmlentities(entry.ssid),
                wifi_stack::toString(entry.authmode),
                wifi_stack::toString(entry.pairwise_cipher),
                wifi_stack::toString(entry.group_cipher),
                entry.rssi,
                entry.primary,
                wifi_stack::toString(wifi_stack::mac_t{entry.bssid}));
        }

        body +=     "</tbody>\n"
                "</table>\n";
    } else {
        body += "<span style=\"color: red;\">no wifi scan result at the moment!</span><br/>\n";
    }

    return ESP_OK;
}

esp_err_t webserver_mqtt_display(httpd_req_t *req, std::string &body)
{
    if (config::enable_mqtt.value()) {
        body += "<br/>\n"
                "<h2>MQTT</h2>\n"
                "<table border=\"1\">\n";
        body += fmt::format("<tr><th>client url</th><td>{}</td></tr>\n", htmlentities(config::broker_url.value()));
        body += fmt::format("<tr><th>client constructed</th><td>{}</td></tr>\n", mqttClient ? "true" : "false");
        if (mqttClient) {
            body += fmt::format("<tr><th>client started</th><td>{}</td></tr>\n", mqttStarted ? "true" : "false");
            body += fmt::format("<tr><th>client connected</th><td>{}</td></tr>\n", mqttConnected ? "true" : "false");
        }
        body += "</table>\n";
    }

    return ESP_OK;
}

template<typename T>
std::string webserver_form_for_config(httpd_req_t *req, ConfigWrapper<T> &config, std::string_view query)
{
    return "<span style=\"color: red;\">form not implemented</span>";
}

template<>
std::string webserver_form_for_config(httpd_req_t *req, ConfigWrapper<bool> &config, std::string_view query)
{
    std::string str;

    {
        char valueBufEncoded[256];
        if (const auto result = httpd_query_key_value(query.data(), config.nvsKey(), valueBufEncoded, 256); result == ESP_OK) {
            char valueBuf[257];
            espcpputils::urldecode(valueBuf, valueBufEncoded);

            std::string_view value{valueBuf};

            if (value == "true" || value == "false") {
                if (const auto result = config.writeToFlash(value == "true"))
                    str += "<span style=\"color: green;\">Successfully saved</span>";
                else
                    str += fmt::format("<span style=\"color: red;\">Error while saving: {}</span>", htmlentities(result.error()));
            } else {
                str += fmt::format("<span style=\"color: red;\">Error while saving: Invalid value \"{}\"</span>", htmlentities(value));
            }
        } else if (result != ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "httpd_query_key_value() %s failed with %s", config.nvsKey(), esp_err_to_name(result));
            str += fmt::format("<span style=\"color: red;\">Error while saving: httpd_query_key_value() failed with {}</span>", esp_err_to_name(result));
        }
    }

    str += fmt::format("<form>"
                           "<input type=\"hidden\" name=\"{}\" value=\"false\" />"
                           "<input type=\"checkbox\" name=\"{}\" value=\"true\"{} />"
                           "<button type=\"submit\">Save</button>"
                       "</form>",
                       htmlentities(config.nvsKey()),
                       htmlentities(config.nvsKey()),
                       config.value() ? " checked" : "");

    return str;
}

template<>
std::string webserver_form_for_config(httpd_req_t *req, ConfigWrapper<std::string> &config, std::string_view query)
{
    std::string str;

    {
        char valueBufEncoded[256];
        if (const auto result = httpd_query_key_value(query.data(), config.nvsKey(), valueBufEncoded, 256); result == ESP_OK) {
            char valueBuf[257];
            espcpputils::urldecode(valueBuf, valueBufEncoded);

            if (const auto result = config.writeToFlash(valueBuf))
                str += "<span style=\"color: green;\">Successfully saved</span>";
            else
                str += fmt::format("<span style=\"color: red;\">Error while saving: {}</span>", htmlentities(result.error()));
        } else if (result != ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "httpd_query_key_value() %s failed with %s", config.nvsKey(), esp_err_to_name(result));
            str += fmt::format("<span style=\"color: red;\">Error while saving: httpd_query_key_value() failed with {}</span>", esp_err_to_name(result));
        }
    }

    str += fmt::format("<form>"
                           "<input type=\"text\" name=\"{}\" value=\"{}\" />"
                           "<button type=\"submit\">Save</button>"
                       "</form>",
                       htmlentities(config.nvsKey()),
                       htmlentities(config.value()));

    return str;
}

template<>
std::string webserver_form_for_config(httpd_req_t *req, ConfigWrapper<gpio_num_t> &config, std::string_view query)
{
    std::string str;

    {
        char valueBufEncoded[256];
        if (const auto result = httpd_query_key_value(query.data(), config.nvsKey(), valueBufEncoded, 256); result == ESP_OK) {
            char valueBuf[257];
            espcpputils::urldecode(valueBuf, valueBufEncoded);

            std::string_view value{valueBuf};

            if (auto parsed = cpputils::fromString<std::underlying_type_t<gpio_num_t>>(value)) {
                if (const auto result = config.writeToFlash(gpio_num_t(*parsed)))
                    str += "<span style=\"color: green;\">Successfully saved</span>";
                else
                    str += fmt::format("<span style=\"color: red;\">Error while saving: {}</span>", htmlentities(result.error()));
            } else {
                str += fmt::format("<span style=\"color: red;\">Error while saving: Invalid value \"{}\"</span>", htmlentities(value));
            }
        } else if (result != ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "httpd_query_key_value() %s failed with %s", config.nvsKey(), esp_err_to_name(result));
        }
    }

    str += fmt::format("<form>"
                       "<input type=\"number\" name=\"{}\" value=\"{}\" />"
                       "<button type=\"submit\">Save</button>"
                       "</form>",
                       htmlentities(config.nvsKey()),
                       config.value());

    return str;
}

template<>
std::string webserver_form_for_config(httpd_req_t *req, ConfigWrapper<espchrono::seconds32> &config, std::string_view query)
{
    std::string str;

    {
        char valueBufEncoded[256];
        if (const auto result = httpd_query_key_value(query.data(), config.nvsKey(), valueBufEncoded, 256); result == ESP_OK) {
            char valueBuf[257];
            espcpputils::urldecode(valueBuf, valueBufEncoded);

            std::string_view value{valueBuf};

            if (auto parsed = cpputils::fromString<espchrono::seconds32::rep>(value)) {
                if (const auto result = config.writeToFlash(espchrono::seconds32(*parsed)))
                    str += "<span style=\"color: green;\">Successfully saved</span>";
                else
                    str += fmt::format("<span style=\"color: red;\">Error while saving: {}</span>", htmlentities(result.error()));
            } else {
                str += fmt::format("<span style=\"color: red;\">Error while saving: Invalid value \"{}\"</span>", htmlentities(value));
            }
        } else if (result != ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "httpd_query_key_value() %s failed with %s", config.nvsKey(), esp_err_to_name(result));
        }
    }

    str += fmt::format("<form>"
                       "<input type=\"number\" name=\"{}\" value=\"{}\" />"
                       "<button type=\"submit\">Save</button>"
                       "</form>",
                       htmlentities(config.nvsKey()),
                       config.value().count());

    return str;
}

esp_err_t webserver_config_display(httpd_req_t *req, std::string &body, std::string_view query)
{
    body += "<br/>\n"
            "<h2>config</h2>\n"
            "<table border=\"1\">\n"
                "<thead>\n"
                    "<tr>\n"
                        "<th>name</th>\n"
                        //"<th>current value</th>\n"
                        "<th>set new value</th>\n"
                        "<th>value in flash</th>\n"
                    "</tr>\n"
                "</thead>\n"
                "<tbody>\n";

    config::foreachConfig([&](auto &config){
        using cpputils::toString;
        using espchrono::toString;

        body += fmt::format("<tr>\n"
                                "<th>{}</th>\n"
                                //"<td>{}</td>\n"
                                "<td>{}</td>\n"
                                "<td>{}</td>\n"
                            "</tr>\n",
                            htmlentities(config.name()),
                            //htmlentities(toString(config.value())),
                            webserver_form_for_config(req, config, query),
                            [&]() -> std::string {
                                if (const auto result = config.readFromFlash()) {
                                    if (*result)
                                        return htmlentities(toString(**result));
                                    else
                                        return "<span style=\"color: grey;\">not set</span>";
                                } else
                                    return fmt::format("<span style=\"color: red\">{}</span>", htmlentities(result.error()));
                            }());
    });

    body +=     "</tbody>\n"
            "</table>\n";

    return ESP_OK;
}

esp_err_t webserver_on_handler(httpd_req_t *req)
{
    if (!config::enable_lamp.value()) {
        ESP_LOGW(TAG, "lamp support not enabled!");
        CALL_AND_EXIT(httpd_resp_send_err, req, HTTPD_400_BAD_REQUEST, "lamp support not enabled!")
    }

    const bool state = (lampState = true);
    writeLamp(state);

    if (mqttConnected)
        mqttVerbosePub(config::topic_lamp_status.value(), state ? "ON" : "OFF", 0, 1);

    std::string_view body{"ON called..."};
    CALL_AND_EXIT_ON_ERROR(httpd_resp_set_type, req, "text/html")
    CALL_AND_EXIT(httpd_resp_send, req, body.data(), body.size())
}

esp_err_t webserver_off_handler(httpd_req_t *req)
{
    if (!config::enable_lamp.value()) {
        ESP_LOGW(TAG, "lamp support not enabled!");
        CALL_AND_EXIT(httpd_resp_send_err, req, HTTPD_400_BAD_REQUEST, "lamp support not enabled!")
    }

    const bool state = (lampState = false);
    writeLamp(state);

    if (mqttConnected)
        mqttVerbosePub(config::topic_lamp_status.value(), state ? "ON" : "OFF", 0, 1);

    std::string_view body{"OFF called..."};
    CALL_AND_EXIT_ON_ERROR(httpd_resp_set_type, req, "text/html")
    CALL_AND_EXIT(httpd_resp_send, req, body.data(), body.size())
}

esp_err_t webserver_toggle_handler(httpd_req_t *req)
{
    if (!config::enable_lamp.value()) {
        ESP_LOGW(TAG, "lamp support not enabled!");
        CALL_AND_EXIT(httpd_resp_send_err, req, HTTPD_400_BAD_REQUEST, "lamp support not enabled!")
    }

    const bool state = (lampState = !lampState);
    writeLamp(state);

    if (mqttConnected)
        mqttVerbosePub(config::topic_lamp_status.value(), state ? "ON" : "OFF", 0, 1);

    std::string_view body{"TOGGLE called..."};
    CALL_AND_EXIT_ON_ERROR(httpd_resp_set_type, req, "text/html")
    CALL_AND_EXIT(httpd_resp_send, req, body.data(), body.size())
}

esp_err_t webserver_reboot_handler(httpd_req_t *req)
{
    shouldReboot = true;

    std::string_view body{"REBOOT called..."};
    CALL_AND_EXIT_ON_ERROR(httpd_resp_set_type, req, "text/html")
    CALL_AND_EXIT(httpd_resp_send, req, body.data(), body.size())
}
} // namespace
} // namespace deckenlampe
