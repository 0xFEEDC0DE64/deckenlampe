#include "mymqtt.h"

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
#include "cppmacros.h"

namespace deckenlampe {
espcpputils::mqtt_client mqttClient;
std::atomic<bool> mqttStarted;
std::atomic<bool> mqttConnected;

namespace {
constexpr const char * const TAG = "MQTT";

void mqttEventHandler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
} // namespace

void init_mqtt()
{
    if (!config::enable_mqtt.value())
        return;

    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = config::broker_url.value().data(),
    };

    mqttClient = espcpputils::mqtt_client{&mqtt_cfg};

    if (!mqttClient)
    {
        ESP_LOGE(TAG, "error while initializing mqtt client!");
        return;
    }

    {
        const auto result = mqttClient.register_event(MQTT_EVENT_ANY, mqttEventHandler, NULL);
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "esp_mqtt_client_register_event(): %s", esp_err_to_name(result));
        if (result != ESP_OK)
            return;
    }

    {
        const auto result = mqttClient.start();
        ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "esp_mqtt_client_start(): %s", esp_err_to_name(result));
        if (result != ESP_OK)
            return;
    }

    mqttStarted = true;
}

void update_mqtt()
{
    if (!config::enable_mqtt.value())
        return;
}

int mqttVerbosePub(std::string_view topic, std::string_view value, int qos, int retain)
{
    ESP_LOGD(TAG, "topic=\"%.*s\" value=\"%.*s\"", topic.size(), topic.data(), value.size(), value.data());

    if (!mqttClient)
    {
        ESP_LOGE(TAG, "mqttClient not constructed!");
        return -1;
    }

    const auto pending_msg_id = mqttClient.publish(topic, value, qos, retain);
    if (pending_msg_id < 0)
        ESP_LOGE(TAG, "topic=\"%.*s\" value=\"%.*s\" failed pending_msg_id=%i", topic.size(), topic.data(), value.size(), value.data(), pending_msg_id);
    else
        ESP_LOGI(TAG, "topic=\"%.*s\" value=\"%.*s\" succeeded pending_msg_id=%i", topic.size(), topic.data(), value.size(), value.data(), pending_msg_id);
    return pending_msg_id;
}

namespace {
void mqttEventHandler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    CPP_UNUSED(event_handler_arg)

    const esp_mqtt_event_t *data = reinterpret_cast<const esp_mqtt_event_t *>(event_data);

    switch (esp_mqtt_event_id_t(event_id))
    {
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_ERROR");

        //        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        //        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        //        {
        //            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
        //            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
        //            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
        //            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));

        //        }
        break;

    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_CONNECTED");

        mqttConnected = true;

        if (config::enable_lamp.value())
        {
            mqttVerbosePub(config::topic_lamp_availability.value(), "online", 0, 1);
            mqttVerbosePub(config::topic_lamp_status.value(), lampState.load() ? "ON" : "OFF", 0, 1);
            mqttClient.subscribe(config::topic_lamp_set.value(), 0);
        }

        if (config::enable_switch.value())
        {
            mqttVerbosePub(config::topic_switch_availability.value(), "online", 0, 1);
            mqttVerbosePub(config::topic_switch_status.value(), switchState.load() ? "ON" : "OFF", 0, 1);
        }

        if (config::enable_dht.value())
        {
            mqttVerbosePub(config::topic_dht11_availability.value(), lastDhtValue ? "online" : "offline", 0, 1);
            if (lastDhtValue)
            {
                if (mqttVerbosePub(config::topic_dht11_temperature.value(), fmt::format("{:.1f}", lastDhtValue->temperature), 0, 1) >= 0)
                    last_dht11_temperature_pub = espchrono::millis_clock::now();
                if (mqttVerbosePub(config::topic_dht11_humidity.value(), fmt::format("{:.1f}", lastDhtValue->humidity), 0, 1) >= 0)
                    last_dht11_humidity_pub = espchrono::millis_clock::now();
            }
        }

        if (config::enable_i2c.value() && config::enable_tsl.value())
        {
            mqttVerbosePub(config::topic_tsl2561_availability.value(), lastTslValue ? "online" : "offline", 0, 1);
            if (lastTslValue)
            {
                if (mqttVerbosePub(config::topic_tsl2561_lux.value(), fmt::format("{:.1f}", lastTslValue->lux), 0, 1) >= 0)
                    last_tsl2561_lux_pub = espchrono::millis_clock::now();
            }
        }

        if (config::enable_i2c.value() && config::enable_bmp.value())
        {
            mqttVerbosePub(config::topic_bmp085_availability.value(), lastBmpValue ? "online" : "offline", 0, 1);
            if (lastBmpValue)
            {
                if (mqttVerbosePub(config::topic_bmp085_pressure.value(), fmt::format("{:.1f}", lastBmpValue->pressure), 0, 1) >= 0)
                    last_bmp085_pressure_pub = espchrono::millis_clock::now();
                if (mqttVerbosePub(config::topic_bmp085_temperature.value(), fmt::format("{:.1f}", lastBmpValue->temperature), 0, 1) >= 0)
                    last_bmp085_temperature_pub = espchrono::millis_clock::now();
                if (mqttVerbosePub(config::topic_bmp085_altitude.value(), fmt::format("{:.1f}", lastBmpValue->altitude), 0, 1) >= 0)
                    last_bmp085_altitude_pub = espchrono::millis_clock::now();
            }
        }

        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_DISCONNECTED");

        mqttConnected = false;

        break;

    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_SUBSCRIBED");
        break;

    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_UNSUBSCRIBED");
        break;

    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_PUBLISHED");
        break;

    case MQTT_EVENT_DATA: {
        std::string_view topic{data->topic, (size_t)data->topic_len};
        std::string_view value{data->data, (size_t)data->data_len};

        ESP_LOGI(TAG, "%s event_id=%s topic=%.*s data=%.*s", event_base, "MQTT_EVENT_DATA", topic.size(), topic.data(), value.size(), value.data());

        if (topic == config::topic_lamp_set.value())
        {
            if (config::enable_lamp.value())
            {
                bool newState = (lampState = (value == "ON"));
                writeLamp(newState);
                if (mqttConnected)
                    mqttVerbosePub(config::topic_lamp_status.value(), newState ? "ON" : "OFF", 0, 1);
            }
            else
            {
                ESP_LOGW(TAG, "received lamp set without lamp support enabled!");
            }
        }
        else
        {
            ESP_LOGW(TAG, "received unknown data topic=%.*s data=%.*s", topic.size(), topic.data(), value.size(), value.data());
        }

        break;
    }
    case MQTT_EVENT_BEFORE_CONNECT:
        ESP_LOGI(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_BEFORE_CONNECT");
        break;

    case MQTT_EVENT_DELETED:
        ESP_LOGI(TAG, "%s event_id=%s", event_base, "MQTT_EVENT_DELETED");
        break;

    default:
        ESP_LOGW(TAG, "%s unknown event_id %i", event_base, event_id);
        break;
    }
}
} // namespace
} // namespace deckenlampe
