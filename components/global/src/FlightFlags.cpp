#include "FlightFlags.hpp"

#include <SerialLogger.hpp>
#include "nvs_flash.h"
#include "nvs.h"

namespace
{
    constexpr const char *TAG = "FlightFlags";

    constexpr const char *NVS_NAMESPACE = "flight";
    constexpr const char *KEY_LAUNCHED = "launched";
    constexpr const char *KEY_ATTEMPTS = "attempts";

    /// @brief Read one byte, return default if error
    uint8_t readU8(const char *key, uint8_t fallback)
    {
        nvs_handle_t handle;
        if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return fallback;

        uint8_t value = fallback;
        const esp_err_t err = nvs_get_u8(handle, key, &value);
        nvs_close(handle);

        if (err != ESP_OK) return fallback;
        return value;
    }

    bool writeU8(const char *key, uint8_t value)
    {
        nvs_handle_t handle;
        esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
        if (err != ESP_OK)
        {
            LOG_ERROR(TAG, "Cannot open NVS to write %s: %s", key, esp_err_to_name(err));
            return false;
        }

        err = nvs_set_u8(handle, key, value);
        if (err == ESP_OK) err = nvs_commit(handle);
        nvs_close(handle);

        if (err != ESP_OK)
        {
            LOG_ERROR(TAG, "Cannot write %s: %s", key, esp_err_to_name(err));
            return false;
        }
        return true;
    }
}

namespace FlightFlags
{
    bool isLaunched()
    {
        return readU8(KEY_LAUNCHED, 0) != 0;
    }

    bool setLaunched(bool launched)
    {
        if (launched && !writeU8(KEY_ATTEMPTS, 0)) return false;

        const bool ok = writeU8(KEY_LAUNCHED, launched ? 1 : 0);
        if (ok) LOG_INFO(TAG, "Launched flag %s", launched ? "set (armed)" : "cleared (recovered)");
        return ok;
    }

    uint8_t deployAttempts()
    {
        return readU8(KEY_ATTEMPTS, 0);
    }

    bool recordDeployAttempt()
    {
        const uint8_t next = static_cast<uint8_t>(deployAttempts() + 1);
        const bool ok = writeU8(KEY_ATTEMPTS, next);
        LOG_WARNING(TAG, "Emergency deploy attempt %u recorded%s", next, ok ? "" : " (NVS save failed)");
        return ok;
    }
}
