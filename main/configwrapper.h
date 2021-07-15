#pragma once

// system includes
#include <string>
#include <cstdint>
#include <utility>

// esp-idf includes
#include <nvs.h>
#include <hal/gpio_types.h>

// 3rdparty lib includes
#include <tl/expected.hpp>
#include <fmt/core.h>

// local includes
#include "espchrono.h"
#include "futurecpp.h"

namespace deckenlampe {
extern nvs_handle_t nvsHandle;

template<typename T> esp_err_t nvs_get(nvs_handle handle, const char* key, T* out_value) = delete;
template<typename T> esp_err_t nvs_set(nvs_handle handle, const char* key, T value) = delete;

template<typename T>
class ConfigWrapper
{
public:
    ConfigWrapper(const char *key, const T &value) : m_key{key}, m_value{value} {}

    const char *key() const { return m_key; }
    const T &value() const { return m_value; }
    void setValue(const T &value) { m_value = value; }

    tl::expected<T, std::string> readFromFlash() const;
    tl::expected<void, std::string> writeToFlash(const T &value);

private:
    const char * const m_key;
    T m_value;
};

template<typename T>
tl::expected<T, std::string> ConfigWrapper<T>::readFromFlash() const
{
    T val;

    const auto result = nvs_get(nvsHandle, m_key, &val);
    //ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "nvs_flash_init(): %s", esp_err_to_name(result));
    if (result == ESP_OK)
        return val;
    else
        return tl::make_unexpected(fmt::format("nvs_get() {} failed with {}", m_key, esp_err_to_name(result)));
}

template<typename T>
tl::expected<void, std::string> ConfigWrapper<T>::writeToFlash(const T &value)
{
    return {};
}

template<> inline esp_err_t nvs_get(nvs_handle handle, const char* key, int8_t* out_value)  { return nvs_get_i8 (handle, key, out_value); }
template<> inline esp_err_t nvs_get(nvs_handle handle, const char* key, uint8_t* out_value) { return nvs_get_u8 (handle, key, out_value); }
template<> inline esp_err_t nvs_get(nvs_handle handle, const char* key, int16_t* out_value) { return nvs_get_i16(handle, key, out_value); }
template<> inline esp_err_t nvs_get(nvs_handle handle, const char* key, uint16_t* out_value){ return nvs_get_u16(handle, key, out_value); }
template<> inline esp_err_t nvs_get(nvs_handle handle, const char* key, int32_t* out_value) { return nvs_get_i32(handle, key, out_value); }
template<> inline esp_err_t nvs_get(nvs_handle handle, const char* key, uint32_t* out_value){ return nvs_get_u32(handle, key, out_value); }
template<> inline esp_err_t nvs_get(nvs_handle handle, const char* key, int64_t* out_value) { return nvs_get_i64(handle, key, out_value); }
template<> inline esp_err_t nvs_get(nvs_handle handle, const char* key, uint64_t* out_value){ return nvs_get_u64(handle, key, out_value); }
template<> inline esp_err_t nvs_get(nvs_handle handle, const char* key, bool* out_value)
{
    uint8_t temp;
    const auto result = nvs_get(handle, key, &temp);
    if (result == ESP_OK && out_value)
        *out_value = temp;
    return result;
}
template<> inline esp_err_t nvs_get(nvs_handle handle, const char* key, gpio_num_t* out_value)
{
    std::underlying_type_t<gpio_num_t> temp;
    const auto result = nvs_get(handle, key, &temp);
    if (result == ESP_OK && out_value)
        *out_value = gpio_num_t(temp);
    return result;
}
template<> inline esp_err_t nvs_get(nvs_handle handle, const char* key, float* out_value)
{
    uint32_t temp;
    const auto result = nvs_get(handle, key, &temp);
    if (result == ESP_OK)
        *out_value = std::bit_cast<float>(temp);
    return result;
}
template<> inline esp_err_t nvs_get(nvs_handle handle, const char* key, double* out_value)
{
    uint64_t temp;
    const auto result = nvs_get(handle, key, &temp);
    if (result == ESP_OK)
        *out_value = std::bit_cast<double>(temp);
    return result;
}
template<> inline esp_err_t nvs_get(nvs_handle handle, const char* key, std::string* out_value)
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

template<> inline esp_err_t nvs_set(nvs_handle handle, const char* key, int8_t value)                    { return nvs_set_i8 (handle, key, value); }
template<> inline esp_err_t nvs_set(nvs_handle handle, const char* key, uint8_t value)                   { return nvs_set_u8 (handle, key, value); }
template<> inline esp_err_t nvs_set(nvs_handle handle, const char* key, int16_t value)                   { return nvs_set_i16(handle, key, value); }
template<> inline esp_err_t nvs_set(nvs_handle handle, const char* key, uint16_t value)                  { return nvs_set_u16(handle, key, value); }
template<> inline esp_err_t nvs_set(nvs_handle handle, const char* key, int32_t value)                   { return nvs_set_i32(handle, key, value); }
template<> inline esp_err_t nvs_set(nvs_handle handle, const char* key, uint32_t value)                  { return nvs_set_u32(handle, key, value); }
template<> inline esp_err_t nvs_set(nvs_handle handle, const char* key, int64_t value)                   { return nvs_set_i64(handle, key, value); }
template<> inline esp_err_t nvs_set(nvs_handle handle, const char* key, uint64_t value)                  { return nvs_set_u64(handle, key, value); }
template<> inline esp_err_t nvs_set(nvs_handle handle, const char* key, bool value)                      { return nvs_set_u8 (handle, key, value); }
template<> inline esp_err_t nvs_set(nvs_handle handle, const char* key, gpio_num_t value)                { return nvs_set    (handle, key, std::to_underlying(value)); }
template<> inline esp_err_t nvs_set(nvs_handle handle, const char* key, float value)                     { return nvs_set(handle, key, std::bit_cast<uint32_t>(value)); }
template<> inline esp_err_t nvs_set(nvs_handle handle, const char* key, double value)                    { return nvs_set(handle, key, std::bit_cast<uint64_t>(value)); }
template<> inline esp_err_t nvs_set(nvs_handle handle, const char* key, const std::string &value)        { return nvs_set_str(handle, key, value.c_str()); }


#define IMPLEMENT_NVS_GET_SET_CHRONO(Name) \
    template<> inline esp_err_t nvs_get(nvs_handle handle, const char* key, Name* out_value) \
    { \
        Name::rep temp; \
        const auto result = nvs_get(handle, key, &temp); \
        if (result == ESP_OK && out_value) \
            *out_value = Name{temp}; \
        return result; \
    } \
    \
    template<> inline esp_err_t nvs_set(nvs_handle handle, const char* key, Name value)                  { return nvs_set(handle, key, Name::rep(value.count())); }

IMPLEMENT_NVS_GET_SET_CHRONO(espchrono::milliseconds32)
IMPLEMENT_NVS_GET_SET_CHRONO(espchrono::seconds32)
IMPLEMENT_NVS_GET_SET_CHRONO(espchrono::minutes32)
IMPLEMENT_NVS_GET_SET_CHRONO(espchrono::hours32)
#undef IMPLEMENT_NVS_GET_SET_CHRONO
} // namespace deckenlampe
