#pragma once

// system includes
#include <string>
#include <cstdint>
#include <utility>

// esp-idf includes
#include <nvs.h>
#include <hal/gpio_types.h>

// local includes
#include "futurecpp.h"
#include "espchrono.h"

namespace deckenlampe {
inline esp_err_t nvs_get(nvs_handle handle, const char* key, int8_t* out_value)  { return nvs_get_i8 (handle, key, out_value); }
inline esp_err_t nvs_get(nvs_handle handle, const char* key, uint8_t* out_value) { return nvs_get_u8 (handle, key, out_value); }
inline esp_err_t nvs_get(nvs_handle handle, const char* key, int16_t* out_value) { return nvs_get_i16(handle, key, out_value); }
inline esp_err_t nvs_get(nvs_handle handle, const char* key, uint16_t* out_value){ return nvs_get_u16(handle, key, out_value); }
inline esp_err_t nvs_get(nvs_handle handle, const char* key, int32_t* out_value) { return nvs_get_i32(handle, key, out_value); }
inline esp_err_t nvs_get(nvs_handle handle, const char* key, uint32_t* out_value){ return nvs_get_u32(handle, key, out_value); }
inline esp_err_t nvs_get(nvs_handle handle, const char* key, int64_t* out_value) { return nvs_get_i64(handle, key, out_value); }
inline esp_err_t nvs_get(nvs_handle handle, const char* key, uint64_t* out_value){ return nvs_get_u64(handle, key, out_value); }
inline esp_err_t nvs_get(nvs_handle handle, const char* key, bool* out_value)
{
    uint8_t temp;
    const auto result = nvs_get(handle, key, &temp);
    if (result == ESP_OK && out_value)
        *out_value = temp;
    return result;
}
inline esp_err_t nvs_get(nvs_handle handle, const char* key, gpio_num_t* out_value)
{
    std::underlying_type_t<gpio_num_t> temp;
    const auto result = nvs_get(handle, key, &temp);
    if (result == ESP_OK && out_value)
        *out_value = gpio_num_t(temp);
    return result;
}
inline esp_err_t nvs_get(nvs_handle handle, const char* key, float* out_value)
{
    uint32_t temp;
    const auto result = nvs_get(handle, key, &temp);
    if (result == ESP_OK)
        *out_value = std::bit_cast<float>(temp);
    return result;
}
inline esp_err_t nvs_get(nvs_handle handle, const char* key, double* out_value)
{
    uint64_t temp;
    const auto result = nvs_get(handle, key, &temp);
    if (result == ESP_OK)
        *out_value = std::bit_cast<double>(temp);
    return result;
}
inline esp_err_t nvs_get(nvs_handle handle, const char* key, std::string* out_value)
{
    size_t length;
    if (const esp_err_t result = nvs_get_str(handle, key, nullptr, &length); result != ESP_OK)
        return result;

    char buf[length];
    if (const esp_err_t result = nvs_get_str(handle, key, buf, &length); result != ESP_OK)
        return result;

    *out_value = buf;

    return ESP_OK;
}

inline esp_err_t nvs_set(nvs_handle handle, const char* key, int8_t value)                    { return nvs_set_i8 (handle, key, value); }
inline esp_err_t nvs_set(nvs_handle handle, const char* key, uint8_t value)                   { return nvs_set_u8 (handle, key, value); }
inline esp_err_t nvs_set(nvs_handle handle, const char* key, int16_t value)                   { return nvs_set_i16(handle, key, value); }
inline esp_err_t nvs_set(nvs_handle handle, const char* key, uint16_t value)                  { return nvs_set_u16(handle, key, value); }
inline esp_err_t nvs_set(nvs_handle handle, const char* key, int32_t value)                   { return nvs_set_i32(handle, key, value); }
inline esp_err_t nvs_set(nvs_handle handle, const char* key, uint32_t value)                  { return nvs_set_u32(handle, key, value); }
inline esp_err_t nvs_set(nvs_handle handle, const char* key, int64_t value)                   { return nvs_set_i64(handle, key, value); }
inline esp_err_t nvs_set(nvs_handle handle, const char* key, uint64_t value)                  { return nvs_set_u64(handle, key, value); }
inline esp_err_t nvs_set(nvs_handle handle, const char* key, bool value)                      { return nvs_set_u8 (handle, key, value); }
inline esp_err_t nvs_set(nvs_handle handle, const char* key, gpio_num_t value)                { return nvs_set    (handle, key, std::to_underlying(value)); }
inline esp_err_t nvs_set(nvs_handle handle, const char* key, float value)                     { return nvs_set(handle, key, std::bit_cast<uint32_t>(value)); }
inline esp_err_t nvs_set(nvs_handle handle, const char* key, double value)                    { return nvs_set(handle, key, std::bit_cast<uint64_t>(value)); }
inline esp_err_t nvs_set(nvs_handle handle, const char* key, const std::string &value)        { return nvs_set_str(handle, key, value.c_str()); }


#define IMPLEMENT_NVS_GET_SET_CHRONO(Name) \
    inline esp_err_t nvs_get(nvs_handle handle, const char* key, Name* out_value) \
    { \
        Name::rep temp; \
        const auto result = nvs_get(handle, key, &temp); \
        if (result == ESP_OK && out_value) \
            *out_value = Name{temp}; \
        return result; \
    } \
    \
    inline esp_err_t nvs_set(nvs_handle handle, const char* key, Name value)                  { return nvs_set(handle, key, Name::rep(value.count())); }

IMPLEMENT_NVS_GET_SET_CHRONO(espchrono::milliseconds32)
IMPLEMENT_NVS_GET_SET_CHRONO(espchrono::seconds32)
IMPLEMENT_NVS_GET_SET_CHRONO(espchrono::minutes32)
IMPLEMENT_NVS_GET_SET_CHRONO(espchrono::hours32)
#undef IMPLEMENT_NVS_GET_SET_CHRONO
} // namespace deckenlampe
