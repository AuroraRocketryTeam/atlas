#include "TelemetryTask.hpp"
#include "esp_system.h"
#include <utils.h>

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
                             std::shared_ptr<EspNowTransmitter> espNowTransmitter,
                             uint32_t intervalMs,
                             IStateMachine* fsm)
    : BaseTask("TelemetryTask"),
      _rocketModel(rocketModel),
      _transmitter(espNowTransmitter),
      _fsm(fsm),
      _transmitIntervalMs(intervalMs),
      _lastTransmitTime(0),
      _lastAckCommandId(0),
      _messagesCreated(0),
      _packetsSent(0),
      _transmitErrors(0)
{
    LOG_INFO("Telemetry", "Created with transmit interval: %lu ms", _transmitIntervalMs);
    LOG_INFO("Telemetry", "Telemetry packet size: %d bytes", sizeof(TelemetryPacket));
}

void TelemetryTask::onTaskStart()
{
    LOG_INFO("Telemetry", "Task started with stack: %u bytes", config.stackSize);
    LOG_INFO("Telemetry", "Transmitter: %s", _transmitter ? "OK" : "NULL");
    _lastTransmitTime = Utils::millis();
}

void TelemetryTask::onTaskStop()
{
    LOG_INFO("Telemetry", "Task stopped - Stats: messages=%lu, packets=%lu, errors=%lu",
             _messagesCreated, _packetsSent, _transmitErrors);
}

void TelemetryTask::taskFunction()
{
    uint32_t loopCount = 0;

    while (running)
    {
        esp_task_wdt_reset();

        // Check early for fast exit
        if (!running)
            break;

        uint32_t now = Utils::millis();

        // Check if it's time to transmit
        if (now - _lastTransmitTime >= _transmitIntervalMs)
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

                LOG_DEBUG("Telemetry", "Packet size: %d bytes", message.size());

                // Transmit via ESP-NOW
                if (transmitMessage(message))
                {
                    _messagesCreated++;
                    LOG_EVERY_MS(5000, INFO, "Telemetry", "Packet %lu transmitted via ESP-NOW", _messagesCreated);
                }
                else
                {
                    _transmitErrors++;
                    LOG_WARNING("Telemetry", "Failed to transmit packet via ESP-NOW (errors: %lu)", _transmitErrors);
                }

                // Transmit via LoRa
                if (_loraTransmitter && running)
                {
                    auto result = _loraTransmitter->transmit(message);
                    if (result.getCode() == E220_SUCCESS)
                    {
                        LOG_EVERY_MS(5000, INFO, "Telemetry", "Packet %lu transmitted via LoRa", _messagesCreated);
                    }
                    else
                    {
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
            uint32_t txSent, txFailed;
            if (_transmitter)
            {
                _transmitter->getStats(txSent, txFailed);
                LOG_INFO("Telemetry", "Stats: msgs=%lu, pkts=%lu, tx_ok=%lu, tx_fail=%lu, heap=%u",
                         _messagesCreated, _packetsSent, txSent, txFailed, esp_get_free_heap_size());
            }
        }

        loopCount++;

        // Check running flag frequently during delay (50ms chunks)
        uint32_t delayRemaining = _transmitIntervalMs / 10; // Split into 10 chunks
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
            LOG_DEBUG("Telemetry", "ACC_X: %.2f, ACC_Y: %.2f, ACC_Z: %.2f", packet.imu.accel_x, packet.imu.accel_y, packet.imu.accel_z);
            packet.imu.gyro_x = outBnoData.orientation_x;
            packet.imu.gyro_y = outBnoData.orientation_y;
            packet.imu.gyro_z = outBnoData.orientation_z;
        } else {
            LOG_EVERY_MS(5000, WARNING, "Telemetry", "BNO055 data not available");
        }

        PressureSensorData outMs56Data1;
        SensorReadStatus baro1Status = _rocketModel->getMS561101BA03Data_1(outMs56Data1);
        if (baro1Status == SensorReadStatus::OK) {
            packet.baro1.pressure = outMs56Data1.pressure;
            packet.baro1.temperature = outMs56Data1.temperature;
            packet.baro_altitude = relAltitude_tele(outMs56Data1.pressure,
                                                    _rocketModel->getLaunchpadBasePressure(),
                                                    _rocketModel->getLaunchpadBaseTemperature());
        } else {
            LOG_EVERY_MS(5000, WARNING, "Telemetry", "Barometer 1 data not available");
        }

        PressureSensorData outMs56Data2;
        SensorReadStatus baro2Status = _rocketModel->getMS561101BA03Data_2(outMs56Data2);
        if (baro2Status == SensorReadStatus::OK) {
            packet.baro2.pressure = outMs56Data2.pressure;
            packet.baro2.temperature = outMs56Data2.temperature;
        } else {
            LOG_EVERY_MS(5000, WARNING, "Telemetry", "Barometer 2 data not available");
        }

        GPSData gpsData;
        SensorReadStatus gpsStatus = _rocketModel->getGPSData(gpsData);
        if (gpsStatus == SensorReadStatus::OK) {
            packet.gps.latitude = gpsData.latitude;
            packet.gps.longitude = gpsData.longitude;
            packet.gps.altitude = gpsData.altitude;
            LOG_DEBUG("Telemetry", "GPS ALT: %.2f LAT: %.6f LON: %.6f",
                      packet.gps.altitude, packet.gps.latitude, packet.gps.longitude);
        } else {
            LOG_EVERY_MS(5000, WARNING, "Telemetry", "GPS data not available");
        }

        packet.velocity = _rocketModel->getHeightGainSpeed();
    }
    catch (const std::exception &e)
    {
        LOG_ERROR("Telemetry", "Exception collecting data: %s", e.what());
        return false;
    }

    return true;
}

bool TelemetryTask::transmitMessage(const std::vector<uint8_t> &message)
{
    if (!_transmitter || message.empty())
    {
        return false;
    }

    // Divide message into packets
    std::vector<Packet> packets = PacketManager::divideMessage(message.data(), message.size());

    if (packets.empty())
    {
        LOG_ERROR("Telemetry", "Failed to divide message into packets");
        return false;
    }

    LOG_DEBUG("Telemetry", "Transmitting %d packets", packets.size());

    // Send each packet
    bool allSuccess = true;
    for (size_t i = 0; i < packets.size() && running; i++)
    {
        ResponseStatusContainer result = _transmitter->transmit(packets[i]);

        if (result.getCode() == 0)
        {
            _packetsSent++;
        }
        else
        {
            LOG_WARNING("Telemetry", "Packet %d/%d failed: %s",
                        i + 1, packets.size(), result.getDescription().c_str());
            allSuccess = false;
            // Continue sending remaining packets even if one fails
        }

        // Small delay between packets to avoid overwhelming receiver
        if (i < packets.size() - 1 && running)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    return allSuccess;
}

void TelemetryTask::pollLoRaRx()
{
    LOG_DEBUG("Telemetry", "Polling LoRa RX for commands");
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

void TelemetryTask::getStats(uint32_t &messages, uint32_t &packets, uint32_t &errors) const
{
    messages = _messagesCreated;
    packets = _packetsSent;
    errors = _transmitErrors;
}
