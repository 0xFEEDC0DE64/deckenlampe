#pragma once

// system includes
#include <string>

// 3rdparty lib includes
#include <tl/expected.hpp>
#include <fmt/core.h>

// local includes
#include "nvswrappers.h"
#include "refwhenneeded.h"

namespace deckenlampe {
extern nvs_handle_t nvsHandle;

template<typename T>
class ConfigWrapper
{
public:
    ConfigWrapper(const char *name, const char *nvsKey, typename cpputils::RefWhenNeeded<T>::T value) :
        m_name{name},
        m_nvsKey{nvsKey},
        m_value{value}
    {}

    const char *name() const { return m_name; }
    const char *nvsKey() const { return m_nvsKey; }
    typename cpputils::RefWhenNeeded<T>::T value() const { return m_value; }
    void setValue(typename cpputils::RefWhenNeeded<T>::T value) { m_value = value; }

    tl::expected<std::optional<T>, std::string> readFromFlash() const;
    tl::expected<void, std::string> writeToFlash(typename cpputils::RefWhenNeeded<T>::T value);

private:
    const char * const m_name;
    const char * const m_nvsKey;
    T m_value;
};

template<typename T>
tl::expected<std::optional<T>, std::string> ConfigWrapper<T>::readFromFlash() const
{
    T val;

    const auto result = nvs_get(nvsHandle, m_nvsKey, &val);
    //ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "nvs_get() %s: %s", m_nvsKey, esp_err_to_name(result));
    if (result == ESP_OK)
        return val;
    else if (result == ESP_ERR_NVS_NOT_FOUND)
        return std::nullopt;
    else
        return tl::make_unexpected(fmt::format("nvs_get() {} failed with {}", m_nvsKey, esp_err_to_name(result)));
}

template<typename T>
tl::expected<void, std::string> ConfigWrapper<T>::writeToFlash(typename cpputils::RefWhenNeeded<T>::T value)
{
    const auto result = nvs_set(nvsHandle, m_nvsKey, value);
    //ESP_LOG_LEVEL_LOCAL((result == ESP_OK ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG, "nvs_set() %s: %s", m_nvsKey, esp_err_to_name(result));
    if (result == ESP_OK)
        return {};
    else
        return tl::make_unexpected(fmt::format("nvs_set() {} failed with {}", m_nvsKey, esp_err_to_name(result)));
}
} // namespace deckenlampe
