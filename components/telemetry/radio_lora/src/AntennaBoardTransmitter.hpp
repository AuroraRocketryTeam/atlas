#pragma once

#include "HostLinkFrame.hpp"

#include <ITransmitter.hpp>
#include <Packet.hpp>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <nlohmann/json.hpp>
#include <string>
#include <variant>
#include <vector>

using TransmitDataType = std::variant<char *, String, std::string, nlohmann::json, std::vector<uint8_t>>;

/**
 * @brief Transmitter that talks to the Antenna Board over SPI
 */
class AntennaBoardTransmitter : public ITransmitter<TransmitDataType>
{
public:
    AntennaBoardTransmitter();
    ~AntennaBoardTransmitter();

    /**
     * @brief Start the SPI bus and queue the first transaction
     */
    ResponseStatusContainer init() override;

    /**
     * @brief Transmit a payload
     * @param data Telemetry payload
     */
    ResponseStatusContainer transmit(TransmitDataType data) override;

    /**
     * @brief Ask the antenna board whether a command arrived from the ground station
     *
     * @param packet Where to write the command
     * @return True when one was waiting hence we received it
     */
    bool receive(CommandPacket *packet);

    /// @brief Status of the antenna board, GPS fix included
    const HostLinkStatus &getStatus() const { return _status; }

    /// @brief True when an exchange happened recently
    bool isLinked() const;

    uint32_t getExchanges() const { return _exchanges; }

private:
    bool exchange(const HostLinkFrame &out);
    void handleIncoming(const HostLinkFrame &frame);

    uint8_t *_txBuffer = nullptr;
    uint8_t *_rxBuffer = nullptr;
    spi_device_handle_t _device = nullptr;
    bool _initialized = false;

    HostLinkStatus _status = {};
    CommandPacket _command = {};
    bool _commandPending = false;
    uint8_t _sequence = 0;
    uint32_t _exchanges = 0;
    uint32_t _lastExchangeMs = 0;
};
