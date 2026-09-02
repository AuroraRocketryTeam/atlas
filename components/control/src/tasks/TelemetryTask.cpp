#include "TelemetryTask.hpp"
#include "esp_system.h"
#include <cstdio>
#include <utils.h>
#include "RuntimeConfig.hpp"

static constexpr uint32_t TELEMETRY_SENSOR_LOG_PERIOD_MS = 500;

constexpr float TROPOSPHERE_HEIGHT = 11000.f; // Troposphere height [m]
constexpr float a = 0.0065f;                  // Troposphere temperature gradient [deg/m]
constexpr float R = 287.05f;                  // Air gas constant [J/Kg/K]
#define n (GRAVITY / (R * a))
#define nInv ((R * a) / GRAVITY)

float relAltitude_tele(float pressure, float pressureRef = 101325.0f,
                       float temperatureRef = 288.15f)
{
    return temperatureRef / a * (1 - powf(pressure / pressureRef, nInv));
}


TelemetryTask::TelemetryTask(std::shared_ptr<RocketModel> rocketModel,
                             IStateMachine* fsm)
    : BaseTask("TelemetryTask"),
      _rocketModel(rocketModel),
      _fsm(fsm),
      _lastTransmitTime(0),
      _lastAckCommandId(0),
      _messagesCreated(0),
      _transmitErrors(0)
{
    LOG_INFO("Telemetry", "Telemetry packet size: %d bytes", sizeof(TelemetryPacket));
}

void TelemetryTask::onTaskStart()
{
    LOG_INFO("Telemetry", "Task started with stack: %u bytes", config.stackSize);
    LOG_INFO("Telemetry", "LoRa transmitter: %s", _loraTransmitter ? "OK" : "NULL");
    _lastTransmitTime = Utils::millis();
}

void TelemetryTask::onTaskStop()
{
    LOG_INFO("Telemetry", "Task stopped - Stats: messages=%lu, errors=%lu",
             _messagesCreated, _transmitErrors);
}

void TelemetryTask::taskFunction()
{
    uint32_t loopCount = 0;
    const RuntimeConfig runtimeConfig = runtime_config_get_flight_snapshot();
    const uint32_t transmitIntervalMs = runtimeConfig.telemetry.period_ms;
    LOG_INFO("Telemetry", "RuntimeConfig telemetry period: %lu ms", static_cast<unsigned long>(transmitIntervalMs));

    while (running)
    {
        esp_task_wdt_reset();

        // Check early for fast exit
        if (!running)
            break;

        uint32_t now = Utils::millis();

        // Check if it's time to transmit
        if (now - _lastTransmitTime >= transmitIntervalMs)
        {
            _lastTransmitTime = now;

            // Poll LoRa RX before building the packet so any received command
            // is reflected in last_ack_command_id within the same TX cycle.
            if (_loraTransmitter && running)
                pollLoRaRx();

            // Collect sensor data into binary packet
            TelemetryPacket packet;
            if (collectSensorData(packet) && running)
            {
                // Convert packet to byte array
                std::vector<uint8_t> message(sizeof(TelemetryPacket));
                memcpy(message.data(), &packet, sizeof(TelemetryPacket));

                // LOG_DEBUG("Telemetry", "Packet size: %d bytes", message.size());

                // Transmit via LoRa
                if (_loraTransmitter && running)
                {
                    auto result = _loraTransmitter->transmit(message);
                    if (result.getCode() == E220_SUCCESS)
                    {
                        _messagesCreated++;
                        LOG_EVERY_MS(5000, INFO, "Telemetry", "Packet %lu transmitted via LoRa", _messagesCreated);
                    }
                    else
                    {
                        _transmitErrors++;
                        LOG_WARNING("Telemetry", "LoRa transmit failed: %s", result.getDescription().c_str());
                    }
                }
                else
                {
                    LOG_EVERY_MS(10000, WARNING, "Telemetry", "LoRa transmitter not available, skipping LoRa transmission");
                }
            }
        }

        // Log stats periodically
        if (loopCount % 10 == 0 && loopCount > 0)
        {
            LOG_INFO("Telemetry", "Stats: msgs=%lu, errors=%lu, heap=%u",
                     _messagesCreated, _transmitErrors, esp_get_free_heap_size());
        }

        loopCount++;

        // Check running flag frequently during delay (50ms chunks)
        uint32_t delayRemaining = transmitIntervalMs / 10; // Split into 10 chunks
        if (delayRemaining < 10)
            delayRemaining = 10;

        for (uint32_t i = 0; i < 10 && running; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(delayRemaining));
        }
    }
}

bool TelemetryTask::collectSensorData(TelemetryPacket &packet)
{
    if (!_rocketModel)
    {
        return false;
    }

    try
    {
        // Initialize packet to zeros
        memset(&packet, 0, sizeof(TelemetryPacket));

        // Add timestamp and validity
        packet.timestamp = Utils::millis();
        packet.dataValid = true;
        packet.flight_phase = _fsm ? static_cast<uint8_t>(_fsm->getCurrentState()) : 0;
        packet.last_ack_command_id = _lastAckCommandId;
        _lastAckCommandId = 0;

        IMUData outBnoData;
        SensorReadStatus bnoStatus = _rocketModel->getBNO055Data(outBnoData);
        if (bnoStatus == SensorReadStatus::OK) {
            packet.imu.accel_x = outBnoData.acceleration_x;
            packet.imu.accel_y = outBnoData.acceleration_y;
            packet.imu.accel_z = outBnoData.acceleration_z;
            packet.imu.gyro_x = outBnoData.orientation_x;
            packet.imu.gyro_y = outBnoData.orientation_y;
            packet.imu.gyro_z = outBnoData.orientation_z;
        }

        PressureSensorData outMs56Data1;
        SensorReadStatus baro1Status = _rocketModel->getMS561101BA03Data_1(outMs56Data1);
        if (baro1Status == SensorReadStatus::OK) {
            packet.baro1.pressure = outMs56Data1.pressure;
            packet.baro1.temperature = outMs56Data1.temperature;
            packet.baro_altitude = relAltitude_tele(outMs56Data1.pressure,
                                                    _rocketModel->getLaunchpadBasePressure(),
                                                    _rocketModel->getLaunchpadBaseTemperature());
        }

        PressureSensorData outMs56Data2;
        SensorReadStatus baro2Status = _rocketModel->getMS561101BA03Data_2(outMs56Data2);
        if (baro2Status == SensorReadStatus::OK) {
            packet.baro2.pressure = outMs56Data2.pressure;
            packet.baro2.temperature = outMs56Data2.temperature;
        }

        GPSData gpsData;
        SensorReadStatus gpsStatus = _rocketModel->getGPSData(gpsData);
        if (gpsStatus == SensorReadStatus::OK) {
            packet.gps.latitude = gpsData.latitude;
            packet.gps.longitude = gpsData.longitude;
            packet.gps.altitude = gpsData.altitude;
        }

        packet.velocity = _rocketModel->getHeightGainSpeed();

        // Single periodic readout of everything this packet carries. A sensor
        // that failed to read is reported as "n/a" rather than as a zeroed value.
        logSensorValues(packet, bnoStatus, baro1Status, baro2Status, gpsStatus);
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("Telemetry", "Exception collecting data: %s", e.what());
        return false;
    }

    return true;
}

void TelemetryTask::logSensorValues(const TelemetryPacket &packet,
                                    SensorReadStatus imuStatus,
                                    SensorReadStatus baro1Status,
                                    SensorReadStatus baro2Status,
                                    SensorReadStatus gpsStatus) const
{
    const bool imuOk = imuStatus == SensorReadStatus::OK;
    const bool baro1Ok = baro1Status == SensorReadStatus::OK;
    const bool baro2Ok = baro2Status == SensorReadStatus::OK;
    const bool gpsOk = gpsStatus == SensorReadStatus::OK;

    char imuText[80] = "n/a";
    char baro1Text[64] = "n/a";
    char baro2Text[64] = "n/a";
    char gpsText[64] = "n/a";

    if (imuOk) {
        snprintf(imuText, sizeof(imuText), "a=%.2f/%.2f/%.2f g=%.2f/%.2f/%.2f",
                 packet.imu.accel_x, packet.imu.accel_y, packet.imu.accel_z,
                 packet.imu.gyro_x, packet.imu.gyro_y, packet.imu.gyro_z);
    }
    if (baro1Ok) {
        snprintf(baro1Text, sizeof(baro1Text), "%.1fPa %.2fC alt=%.2fm",
                 packet.baro1.pressure, packet.baro1.temperature, packet.baro_altitude);
    }
    if (baro2Ok) {
        snprintf(baro2Text, sizeof(baro2Text), "%.1fPa %.2fC",
                 packet.baro2.pressure, packet.baro2.temperature);
    }
    if (gpsOk) {
        snprintf(gpsText, sizeof(gpsText), "%.6f,%.6f %.2fm",
                 packet.gps.latitude, packet.gps.longitude, packet.gps.altitude);
    }

    LOG_EVERY_MS(TELEMETRY_SENSOR_LOG_PERIOD_MS, INFO, "Telemetry",
                 "Sensors | IMU %s | Baro1 %s | Baro2 %s | GPS %s | v=%.2fm/s",
                 imuText, baro1Text, baro2Text, gpsText, packet.velocity);
}

void TelemetryTask::pollLoRaRx()
{
    // LOG_DEBUG("Telemetry", "Polling LoRa RX for commands");
    CommandPacket cmd;
    while (_loraTransmitter->receive(&cmd))
    {
        LOG_INFO("Telemetry", "Command received: 0x%02X", cmd.command_id);
        _lastAckCommandId = cmd.command_id;
        handleCommand(static_cast<CommandId>(cmd.command_id));
    }
}

void TelemetryTask::handleCommand(CommandId id)
{
    if (!_fsm) return;

    switch (id)
    {
        case CommandId::PING:
            LOG_INFO("Telemetry", "PING received from ground station");
            break;
        case CommandId::ABORT:
            LOG_WARNING("Telemetry", "ABORT command received! Sending event");
            _fsm->sendEvent(FSMEvent::EMERGENCY_ABORT);
            break;
        case CommandId::EMERG_CHUTE_OPEN:
            LOG_WARNING("Telemetry", "EMERG_CHUTE_OPEN command received! Forcing transition to APOGEE");
            _fsm->forceTransition(RocketState::APOGEE);
            break;
        default:
            LOG_WARNING("Telemetry", "Unknown command id: 0x%02X", id);
            break;
    }
}

void TelemetryTask::getStats(uint32_t &messages, uint32_t &errors) const
{
    messages = _messagesCreated;
    errors = _transmitErrors;
}
