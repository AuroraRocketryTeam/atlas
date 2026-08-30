#pragma once

#include "BaseTask.hpp"
#include <RocketModel.hpp>
#include "E220LoRaTransmitter.hpp"
#include "SerialLogger.hpp"
#include "IStateMachine.hpp"
#include <memory>
#include <cstdint>
#include "esp_task_wdt.h"
#include <Arduino.h>
#include <config.h>

/**
 * @brief Binary telemetry packet structure for efficient transmission.
 *
 * This structure is tightly packed (no padding) for efficient transmission
 * over LoRa. Total size: 67 bytes.
 *
 * All multi-byte values are little-endian (ESP32 native).
 */
#pragma pack(push, 1)
struct TelemetryPacket
{
    uint32_t timestamp; ///< Milliseconds since boot
    bool dataValid;     ///< True if sensor data was successfully collected

    struct
    {
        float accel_x, accel_y, accel_z; ///< Accelerometer (m/s²)
        float gyro_x, gyro_y, gyro_z;    ///< Gyroscope / angular velocity (rad/s)
    } imu;

    struct
    {
        float pressure;    ///< Pressure (hPa)
        float temperature; ///< Temperature (°C)
    } baro1;

    struct
    {
        float pressure;    ///< Pressure (hPa)
        float temperature; ///< Temperature (°C)
    } baro2;

    float baro_altitude; ///< Relative altitude above launch point, calculated from baro (m)

    struct
    {
        float latitude;  ///< Latitude (degrees)
        float longitude; ///< Longitude (degrees)
        float altitude;  ///< GPS altitude (m)
    } gps;

    float velocity; ///< (m/s)

    uint8_t flight_phase; ///< 0 = INACTIVE, 10 = RECOVERED
    uint8_t last_ack_command_id; ///< Last successfully received command (CommandId), 0x00 = none
};
#pragma pack(pop)

/**
 * @brief Task that periodically collects sensor data and transmits it via LoRa.
 *
 * This task reads from SharedSensorData, serializes it to binary format,
 * and transmits it using E220LoRaTransmitter.
 *
 * Transmission rate is configurable via constructor.
 */
class TelemetryTask : public BaseTask
{
private:
    std::shared_ptr<RocketModel> _rocketModel;
    std::shared_ptr<E220LoRaTransmitter> _loraTransmitter;
    IStateMachine* _fsm;

    uint32_t _transmitIntervalMs;
    uint32_t _lastTransmitTime;

    uint8_t _lastAckCommandId;

    // Statistics
    uint32_t _messagesCreated;
    uint32_t _transmitErrors;

public:
    /**
     * @brief Construct a new Telemetry Task.
     *
     * @param rocketModel Shared rocket model to read sensor data from.
     * @param intervalMs Interval between transmissions in milliseconds (default 1000ms = 1Hz).
     * @param fsm State machine used to report the flight phase and dispatch ground commands.
     */
    TelemetryTask(std::shared_ptr<RocketModel> rocketModel,
                  uint32_t intervalMs = 1000,
                  IStateMachine* fsm = nullptr);

    /**
     * @brief Get transmission statistics.
     *
     * @param messages Output: number of messages created.
     * @param errors Output: number of transmission errors.
     */
    void getStats(uint32_t &messages, uint32_t &errors) const;

    /** @brief Set LoRa transmitter. Will be used only if present. */
    void setLoRaTransmitter(std::shared_ptr<E220LoRaTransmitter> transmitter) { _loraTransmitter = transmitter; }

protected:
    void taskFunction() override;
    void onTaskStart() override;
    void onTaskStop() override;

private:
    /**
     * @brief Collect current sensor data into a binary telemetry packet.
     *
     * @param packet Reference to packet structure to fill with sensor data.
     * @return true if data collection successful, false on mutex timeout or error.
     */
    bool collectSensorData(TelemetryPacket &packet);

    /**
     * @brief Poll the LoRa receiver for commands, automatically dispaches them.
     */
    void pollLoRaRx();

    /**
     * @brief Dispatch a received command to the FSM.
     * @param id The command identifier.
     */
    void handleCommand(CommandId id);
};
