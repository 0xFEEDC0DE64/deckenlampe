#include "feature_pms.h"

// esp-idf includes
#include <esp_log.h>

// Arduino includes
#include <Arduino.h>
#include <HardwareSerial.h>

// 3rdparty lib includes
#include <espstrutils.h>
#include <espchrono.h>

// local includes
#include "myconfig.h"
#include "mymqtt.h"

using namespace std::chrono_literals;

namespace deckenlampe {
namespace {
constexpr const char * const TAG = "PMS";

espchrono::millis_clock::time_point lastRequest;

struct DATA {
  // Standard Particles, CF=1
  uint16_t PM_SP_UG_1_0;
  uint16_t PM_SP_UG_2_5;
  uint16_t PM_SP_UG_10_0;

  // Atmospheric environment
  uint16_t PM_AE_UG_1_0;
  uint16_t PM_AE_UG_2_5;
  uint16_t PM_AE_UG_10_0;
} pms_data;

uint8_t pms_payload[12];
uint8_t pms_index = 0;
uint16_t pms_frameLen;
uint16_t pms_checksum;
uint16_t pms_calculatedChecksum;
}

void init_pms()
{
    if (!config::enable_pms.value())
        return;

    ESP_LOGI(TAG, "starting Serial1 for PMS %i %i", config::pins_pms_rx.value(), config::pins_pms_tx.value());

    Serial1.begin(9600, SERIAL_8N1, config::pins_pms_rx.value(), config::pins_pms_tx.value());

    {
        ESP_LOGI(TAG, "setting into active mode...");
        uint8_t command[] = { 0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71 };
        Serial1.write({command, sizeof(command)});
    }
}

void update_pms()
{
    if (!config::enable_pms.value())
        return;

    if (espchrono::ago(lastRequest) >= 1s)
    {
        lastRequest = espchrono::millis_clock::now();

        if (false)
        {
            ESP_LOGI(TAG, "setting into active mode...");
            uint8_t command[] = { 0x42, 0x4D, 0xE1, 0x00, 0x01, 0x01, 0x71 };
            Serial1.write({command, sizeof(command)});
        }

        if (false)
        {
            ESP_LOGI(TAG, "setting into passive mode...");
            uint8_t command[] = { 0x42, 0x4D, 0xE1, 0x00, 0x00, 0x01, 0x70 };
            Serial1.write({command, sizeof(command)});
        }

        if (false)
        {
            ESP_LOGI(TAG, "requesting read...");
            uint8_t command[] = { 0x42, 0x4D, 0xE2, 0x00, 0x00, 0x01, 0x71 };
            Serial1.write({command, sizeof(command)});
        }
    }

    if (false)
    {
        uint8_t buf[256];
        if (const auto read = Serial1.read(buf, sizeof(buf)))
        {
            ESP_LOGI(TAG, "received %zd %s", read, espcpputils::toHexString({buf, read}).c_str());
        }
    }

    if (Serial1.available())
    {
        uint8_t ch = Serial1.read();

        switch (pms_index)
        {
        case 0:
            if (ch != 0x42)
                return;
            pms_calculatedChecksum = ch;
            break;
        case 1:
            if (ch != 0x4D)
            {
                pms_index = 0;
                return;
            }
            pms_calculatedChecksum += ch;
            break;
        case 2:
            pms_calculatedChecksum += ch;
            pms_frameLen = ch << 8;
            break;
        case 3:
            pms_frameLen |= ch;
            // Unsupported sensor, different frame length, transmission error e.t.c.
            if (pms_frameLen != 2 * 9 + 2 && pms_frameLen != 2 * 13 + 2)
            {
                pms_index = 0;
                return;
            }
            pms_calculatedChecksum += ch;
            break;
        default:
            if (pms_index == pms_frameLen + 2)
            {
                pms_checksum = ch << 8;
            }
            else if (pms_index == pms_frameLen + 2 + 1)
            {
                pms_checksum |= ch;

                if (pms_calculatedChecksum == pms_checksum)
                {
                    //_PMSstatus = STATUS_OK;

                    // Standard Particles, CF=1.
                    pms_data.PM_SP_UG_1_0 = makeWord(pms_payload[0], pms_payload[1]);
                    pms_data.PM_SP_UG_2_5 = makeWord(pms_payload[2], pms_payload[3]);
                    pms_data.PM_SP_UG_10_0 = makeWord(pms_payload[4], pms_payload[5]);

                    // Atmospheric Environment.
                    pms_data.PM_AE_UG_1_0 = makeWord(pms_payload[6], pms_payload[7]);
                    pms_data.PM_AE_UG_2_5 = makeWord(pms_payload[8], pms_payload[9]);
                    pms_data.PM_AE_UG_10_0 = makeWord(pms_payload[10], pms_payload[11]);

                    ESP_LOGI(TAG, "PM_SP_UG_1_0 %hu", pms_data.PM_SP_UG_1_0);
                    ESP_LOGI(TAG, "PM_SP_UG_2_5 %hu", pms_data.PM_SP_UG_2_5);
                    ESP_LOGI(TAG, "PM_SP_UG_10_0 %hu", pms_data.PM_SP_UG_10_0);
                    ESP_LOGI(TAG, "PM_AE_UG_1_0 %hu", pms_data.PM_AE_UG_1_0);
                    ESP_LOGI(TAG, "PM_AE_UG_2_5 %hu", pms_data.PM_AE_UG_2_5);
                    ESP_LOGI(TAG, "PM_AE_UG_10_0 %hu", pms_data.PM_AE_UG_10_0);

                    if (mqttConnected)
                    {
                        mqttVerbosePub(config::topic_pms_st_1_0.value(), std::to_string(pms_data.PM_SP_UG_1_0), 0, 1);
                        mqttVerbosePub(config::topic_pms_st_2_5.value(), std::to_string(pms_data.PM_SP_UG_2_5), 0, 1);
                        mqttVerbosePub(config::topic_pms_st_10_0.value(), std::to_string(pms_data.PM_SP_UG_10_0), 0, 1);
                        mqttVerbosePub(config::topic_pms_ae_1_0.value(), std::to_string(pms_data.PM_AE_UG_1_0), 0, 1);
                        mqttVerbosePub(config::topic_pms_ae_2_5.value(), std::to_string(pms_data.PM_AE_UG_2_5), 0, 1);
                        mqttVerbosePub(config::topic_pms_ae_10_0.value(), std::to_string(pms_data.PM_AE_UG_10_0), 0, 1);
                    }
                }

                pms_index = 0;
                return;
            }
            else
            {
                pms_calculatedChecksum += ch;
                uint8_t payloadIndex = pms_index - 4;

                // Payload is common to all sensors (first 2x6 bytes).
                if (payloadIndex < sizeof(pms_payload))
                {
                    pms_payload[payloadIndex] = ch;
                }
            }
            break;
        }

        pms_index++;
    }
}

} // namespace deckenlampe
