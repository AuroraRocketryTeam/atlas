#include "TelemetryTask.hpp"
#include <utils.h>
#include "RuntimeConfig.hpp"
#ifdef CONFIG_TELEMETRY_USB_MIRROR
#  include "driver/usb_serial_jtag.h"
#endif

constexpr float TROPOSPHERE_HEIGHT = 11000.f; // Troposphere height [m]
constexpr float a = 0.0065f;                  // Troposphere temperature gradient [deg/m]
constexpr float R = 287.05f;                  // Air gas constant [J/Kg/K]
#define n (GRAVITY / (R * a))
#define nInv ((R * a) / GRAVITY)

namespace {
std::atomic_bool s_loraAvailable{false};
std::atomic_uint32_t s_loraSuccessfulMessages{0};
std::atomic_uint32_t s_loraFailedMessages{0};
std::atomic_uint32_t s_loraLastSuccessMs{0};
}

float relAltitude_tele(float pressure, float pressureRef = 101325.0f,
                       float temperatureRef = 288.15f)
{
    return temperatureRef / a * (1 - powf(pressure / pressureRef, nInv));
}


TelemetryTask::TelemetryTask(std::shared_ptr<RocketModel> rocketModel,
                             std::shared_ptr<EspNowTransmitter> espNowTransmitter,
                             IStateMachine* fsm)
    : BaseTask("TelemetryTask"),
      _rocketModel(rocketModel),
      _transmitter(espNowTransmitter),
      _fsm(fsm),
      _lastTransmitTime(0),
      _lastAckCommandId(0),
      _messagesCreated(0),
      _packetsSent(0),
      _transmitErrors(0)
{
    LOG_INFO("Telemetry", "Telemetry packet size: %d bytes", sizeof(TelemetryPacket));
}

void TelemetryTask::onTaskStart()
{
    LOG_INFO("Telemetry", "Task started with stack: %u bytes", config.stackSize);
    LOG_INFO("Telemetry", "Transmitter: %s", _transmitter ? "OK" : "NULL");
    _lastTransmitTime = Utils::millis();
    s_loraSuccessfulMessages.store(0, std::memory_order_relaxed);
    s_loraFailedMessages.store(0, std::memory_order_relaxed);
    s_loraLastSuccessMs.store(0, std::memory_order_relaxed);
}

void TelemetryTask::setLoRaTransmitter(std::shared_ptr<E220LoRaTransmitter> transmitter)
{
    _loraTransmitter = std::move(transmitter);
    s_loraAvailable.store(_loraTransmitter != nullptr, std::memory_order_release);
}

void TelemetryTask::onTaskStop()
{
    LOG_INFO("Telemetry", "Task stopped - Stats: messages=%lu, packets=%lu, errors=%lu",
             _messagesCreated, _packetsSent, _transmitErrors);
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

                _messagesCreated++;

                // Transmit via ESP-NOW when the transmitter is available. HIL
                // builds intentionally skip it because HIL owns the SoftAP.
                if (_transmitter)
                {
                    if (transmitMessage(message))
                    {
                        // LOG_INFO("Telemetry", "Packet %lu transmitted via ESP-NOW", _messagesCreated);
                    }
                    else
                    {
                        _transmitErrors++;
                        LOG_WARNING("Telemetry", "Failed to transmit packet via ESP-NOW (errors: %lu)", _transmitErrors);
                    }
                }

                // Transmit via LoRa
                if (_loraTransmitter && running)
                {
                    auto result = _loraTransmitter->transmit(message);
                    if (result.getCode() == E220_SUCCESS)
                    {
                        s_loraSuccessfulMessages.fetch_add(1, std::memory_order_relaxed);
                        s_loraLastSuccessMs.store(now, std::memory_order_release);
                        // LOG_INFO("Telemetry", "Packet %lu transmitted via LoRa", _messagesCreated);
                    }
                    else
                    {
                        s_loraFailedMessages.fetch_add(1, std::memory_order_relaxed);
                        LOG_WARNING("Telemetry", "LoRa transmit failed: %s", result.getDescription().c_str());
                    }
                }
                else
                {
                    // LOG_WARNING("Telemetry", "LoRa transmitter not available, skipping LoRa transmission");
                }

#ifdef CONFIG_TELEMETRY_USB_MIRROR
                if (message.size() <= 255)
                {
                    const uint8_t frame[3] = {0xAA, 0x55, static_cast<uint8_t>(message.size())};
                    usb_serial_jtag_write_bytes(frame, sizeof(frame), portMAX_DELAY);
                    usb_serial_jtag_write_bytes(message.data(), message.size(), portMAX_DELAY);
                }
#endif
            }
        }

        // Log stats periodically
        if (loopCount % 10 == 0 && loopCount > 0)
        {
            uint32_t espNowSent = 0;
            uint32_t espNowFailed = 0;
            if (_transmitter)
            {
                _transmitter->getStats(espNowSent, espNowFailed); // passed by reference
            }
            const LoRaTelemetryStatus lora = getLoRaStatus();
            LOG_INFO("Telemetry", "Stats: msgs=%lu, espnow_pkts=%lu, espnow_errors=%lu, lora_ok=%lu, lora_fail=%lu, heap=%u",
                     _messagesCreated, _packetsSent, _transmitErrors,
                     lora.successful_messages, lora.failed_messages, ESP.getFreeHeap());
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
            // LOG_DEBUG("Telemetry", "ACC_X: %.2f, ACC_Y: %.2f, ACC_Z: %.2f", packet.imu.accel_x, packet.imu.accel_y, packet.imu.accel_z);
            packet.imu.gyro_x = outBnoData.orientation_x;
            packet.imu.gyro_y = outBnoData.orientation_y;
            packet.imu.gyro_z = outBnoData.orientation_z;
        } else {
            LOG_WARNING("Telemetry", "BNO055 data not available");
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
            LOG_WARNING("Telemetry", "Barometer 1 data not available");
        }

        PressureSensorData outMs56Data2;
        SensorReadStatus baro2Status = _rocketModel->getMS561101BA03Data_2(outMs56Data2);
        if (baro2Status == SensorReadStatus::OK) {
            packet.baro2.pressure = outMs56Data2.pressure;
            packet.baro2.temperature = outMs56Data2.temperature;
        } else {
            // LOG_WARNING("Telemetry", "Barometer 2 data not available");
        }

        GPSData gpsData;
        SensorReadStatus gpsStatus = _rocketModel->getGPSData(gpsData);
        if (gpsStatus == SensorReadStatus::OK) {
            packet.gps.latitude = gpsData.latitude;
            packet.gps.longitude = gpsData.longitude;
            packet.gps.altitude = gpsData.altitude;
            // LOG_DEBUG("Telemetry", "GPS ALT: %.2f LAT: %.6f LON: %.6f",
            //           packet.gps.altitude, packet.gps.latitude, packet.gps.longitude);
        } else {
            // LOG_WARNING("Telemetry", "GPS data not available");
        }

        packet.velocity = _rocketModel->getHeightGainSpeed();
        // LOG_INFO("Telemetry", "Done collecting sensor data!");
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

    // LOG_DEBUG("Telemetry", "Transmitting %d packets", packets.size());

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

void TelemetryTask::getStats(uint32_t &messages, uint32_t &packets, uint32_t &errors) const
{
    messages = _messagesCreated;
    packets = _packetsSent;
    errors = _transmitErrors;
}

LoRaTelemetryStatus TelemetryTask::getLoRaStatus()
{
    return {
        s_loraAvailable.load(std::memory_order_acquire),
        s_loraSuccessfulMessages.load(std::memory_order_relaxed),
        s_loraFailedMessages.load(std::memory_order_relaxed),
        s_loraLastSuccessMs.load(std::memory_order_acquire),
    };
}
