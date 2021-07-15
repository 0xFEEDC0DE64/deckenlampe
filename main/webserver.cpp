#include "webserver.h"

// system includes
#include <string_view>
#include <string>

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

namespace deckenlampe {
httpd_handle_t httpdHandle;

namespace {
constexpr const char * const TAG = "WEBSERVER";

esp_err_t webserver_root_handler(httpd_req_t *req);
esp_err_t webserver_on_handler(httpd_req_t *req);
esp_err_t webserver_off_handler(httpd_req_t *req);
esp_err_t webserver_toggle_handler(httpd_req_t *req);
esp_err_t webserver_reboot_handler(httpd_req_t *req);
} // namespace

void init_webserver()
{
    {
        httpd_config_t httpConfig HTTPD_DEFAULT_CONFIG();

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
}

namespace {
#define CALL_AND_EXIT_ON_ERROR(METHOD, ...) \
    if (const auto result = METHOD(__VA_ARGS__); result != ESP_OK) \
    { \
        ESP_LOGE(TAG, "%s() failed with %s", #METHOD, esp_err_to_name(result)); \
        return result; \
    }

#define CALL_AND_EXIT(METHOD, ...) \
    if (const auto result = METHOD(__VA_ARGS__); result != ESP_OK) \
    { \
        ESP_LOGE(TAG, "%s() failed with %s", #METHOD, esp_err_to_name(result)); \
        return result; \
    } \
    else \
        return result;

esp_err_t webserver_root_handler(httpd_req_t *req)
{
    std::string body = "this is work in progress...<br/>\n";

    if (config::enable_lamp) {
        body += "<a href=\"/on\">on</a><br/>\n"
                "<a href=\"/off\">off</a><br/>\n"
                "<a href=\"/toggle\">toggle</a><br/>\n";
    }

    body += "<a href=\"/reboot\">reboot</a><br/>\n"
            "<br/>\n";

    if (config::enable_lamp)
        body += fmt::format("Lamp: {}<br/>\n", lampState ? "ON" : "OFF");

    if (config::enable_switch)
        body += fmt::format("Switch: {}<br/>\n", switchState ? "ON" : "OFF");

    if (config::enable_dht) {
        if (lastDhtValue) {
            body += fmt::format("DHT11 Temperature: {:.1f} C<br/>\n", lastDhtValue->temperature);
            body += fmt::format("DHT11 Humidity: {:.1f} %<br/>\n", lastDhtValue->humidity);
        } else
            body += "DHT11 not available at the moment<br/>\n";
    }

    if (config::enable_tsl) {
        if (lastTslValue) {
            body += fmt::format("TSL2561 Brightness: {:.1f} lux<br/>\n", lastTslValue->lux);
        } else
            body += "TSL2561 not available at the moment<br/>\n";
    }

    if (config::enable_bmp) {
        if (lastBmpValue) {
            body += fmt::format("BMP085 Pressure: {:.1f} lux<br/>\n", lastBmpValue->pressure);
            body += fmt::format("BMP085 Temperature: {:.1f} C<br/>\n", lastBmpValue->temperature);
            body += fmt::format("BMP085 Altitude: {:.1f} m<br/>\n", lastBmpValue->altitude);
        } else
            body += "BMP085 not available at the moment<br/>\n";
    }

    CALL_AND_EXIT_ON_ERROR(httpd_resp_set_type, req, "text/html")
    CALL_AND_EXIT_ON_ERROR(httpd_resp_send, req, body.data(), body.size())

    return ESP_OK;
}

esp_err_t webserver_on_handler(httpd_req_t *req)
{
    if (!config::enable_lamp) {
        ESP_LOGW(TAG, "lamp support not enabled!");
        CALL_AND_EXIT_ON_ERROR(httpd_resp_send_err, req, HTTPD_400_BAD_REQUEST, "lamp support not enabled!")
    }

    const bool state = (lampState = true);
    writeLamp(state);

    if (mqttClient && mqttConnected)
        mqttVerbosePub(config::topic_lamp_status, state ? "ON" : "OFF", 0, 1);

    std::string_view body{"ON called..."};
    CALL_AND_EXIT_ON_ERROR(httpd_resp_set_type, req, "text/html")
    CALL_AND_EXIT_ON_ERROR(httpd_resp_send, req, body.data(), body.size())

    return ESP_OK;
}

esp_err_t webserver_off_handler(httpd_req_t *req)
{
    if (!config::enable_lamp) {
        ESP_LOGW(TAG, "lamp support not enabled!");
        CALL_AND_EXIT_ON_ERROR(httpd_resp_send_err, req, HTTPD_400_BAD_REQUEST, "lamp support not enabled!")
    }

    const bool state = (lampState = false);
    writeLamp(state);

    if (mqttClient && mqttConnected)
        mqttVerbosePub(config::topic_lamp_status, state ? "ON" : "OFF", 0, 1);

    std::string_view body{"OFF called..."};
    CALL_AND_EXIT_ON_ERROR(httpd_resp_set_type, req, "text/html")
    CALL_AND_EXIT_ON_ERROR(httpd_resp_send, req, body.data(), body.size())

    return ESP_OK;
}

esp_err_t webserver_toggle_handler(httpd_req_t *req)
{
    if (!config::enable_lamp) {
        ESP_LOGW(TAG, "lamp support not enabled!");
        CALL_AND_EXIT_ON_ERROR(httpd_resp_send_err, req, HTTPD_400_BAD_REQUEST, "lamp support not enabled!")
    }

    const bool state = (lampState = !lampState);
    writeLamp(state);

    if (mqttClient && mqttConnected)
        mqttVerbosePub(config::topic_lamp_status, state ? "ON" : "OFF", 0, 1);

    std::string_view body{"TOGGLE called..."};
    CALL_AND_EXIT_ON_ERROR(httpd_resp_set_type, req, "text/html")
    CALL_AND_EXIT_ON_ERROR(httpd_resp_send, req, body.data(), body.size())

    return ESP_OK;
}

esp_err_t webserver_reboot_handler(httpd_req_t *req)
{
    std::string_view body{"REBOOT called..."};
    CALL_AND_EXIT_ON_ERROR(httpd_resp_set_type, req, "text/html")
    CALL_AND_EXIT_ON_ERROR(httpd_resp_send, req, body.data(), body.size())

    esp_restart();

    return ESP_OK;
}
} // namespace
} // namespace deckenlampe
