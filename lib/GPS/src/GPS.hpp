#pragma once

#include <ISensor.hpp>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <config.h>

/**
 * @class GPS
 * @brief Driver for u-blox GNSS modules over I2C using the SparkFun library.
 *
 * This class implements the ISensor interface and provides a minimal wrapper
 * around SFE_UBLOX_GNSS to initialize the GNSS receiver and retrieve the
 * most relevant navigation data (fix status, satellite count and, when valid,
 * position and related metrics).
 *
 * Data contract (keys in returned SensorData):
 * - "fix" (uint8_t): 0 no-fix, 2 2D-fix, 3 3D-fix
 * - "satellites" (uint8_t): number of satellites used in solution
 * - "latitude" (float, degrees) when fix >= 3
 * - "longitude" (float, degrees) when fix >= 3
 * - optional: "altitude" (float, meters), "speed" (float, km/h), "hdop" (float)
 */
class GPS : public ISensor
{
public:
    /**
     * @brief Construct a new GPS sensor object.
     */
    GPS();

    /**
     * @brief Initialize the GNSS receiver on the default I2C bus/address.
     *
     * This will call Wire.begin() if needed and probe the module via
     * SFE_UBLOX_GNSS::begin.
     *
     * @return true on successful initialization, false otherwise.
     */
    bool init() override;

    /**
     * @brief Read current GNSS data and expose it as SensorData.
     *
     * On success, always returns a SensorData with at least the fields
     * "fix" and "satellites". When a 3D fix is available (fix >= 3),
     * latitude and longitude (in decimal degrees) are included; additional
     * fields like altitude (m), speed (km/h) and hdop may be provided
     * depending on sensor configuration.
     *
     * @return std::optional<SensorData> Populated data if the read succeeded;
     *         std::nullopt on communication failure.
     */
    std::optional<SensorData> getData() override;

private:
    /**
     * @brief Underlying SparkFun GNSS driver instance.
     */
    SFE_UBLOX_GNSS myGNSS;
};