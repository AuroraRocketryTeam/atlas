#include "sdkconfig.h"

#if CONFIG_TELEMETRY_LINK_ANTENNA_BOARD

#include "AntennaBoardTransmitter.hpp"

#include <SerialLogger.hpp>
#include <board.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <cstring>

static const char *TAG = "AntennaBoard";

// Different from barometer and flash.
static constexpr spi_host_device_t LINK_HOST = SPI3_HOST;
static constexpr int LINK_CLOCK_HZ = 1000000;
static constexpr uint32_t LINK_TIMEOUT_MS = 3000;
static constexpr uint32_t COMMAND_POLL_INTERVAL_MS = CONFIG_ANTENNA_LINK_COMMAND_POLL_MS;

static constexpr gpio_num_t LINK_MOSI = MANNY_ANTENNA_LINK_MOSI_PIN;
static constexpr gpio_num_t LINK_MISO = MANNY_ANTENNA_LINK_MISO_PIN;
static constexpr gpio_num_t LINK_CLK = MANNY_ANTENNA_LINK_CLK_PIN;
static constexpr gpio_num_t LINK_CS = MANNY_ANTENNA_LINK_CS_PIN;

static uint32_t nowMs()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

AntennaBoardTransmitter::AntennaBoardTransmitter() = default;

AntennaBoardTransmitter::~AntennaBoardTransmitter()
{
    if (_device) spi_bus_remove_device(_device);
    if (_initialized) spi_bus_free(LINK_HOST);
    if (_txBuffer) heap_caps_free(_txBuffer);
    if (_rxBuffer) heap_caps_free(_rxBuffer);
}

ResponseStatusContainer AntennaBoardTransmitter::init()
{
    if (_initialized) return ResponseStatusContainer::ok("Already initialized");

    _txBuffer = static_cast<uint8_t *>(heap_caps_malloc(HOST_LINK_FRAME_SIZE, MALLOC_CAP_DMA));
    _rxBuffer = static_cast<uint8_t *>(heap_caps_malloc(HOST_LINK_FRAME_SIZE, MALLOC_CAP_DMA));
    if (_txBuffer == nullptr || _rxBuffer == nullptr)
        return ResponseStatusContainer::error("Cannot allocate the transfer buffers");

    spi_bus_config_t bus = {};
    bus.mosi_io_num = LINK_MOSI;
    bus.miso_io_num = LINK_MISO;
    bus.sclk_io_num = LINK_CLK;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = HOST_LINK_FRAME_SIZE;

    esp_err_t err = spi_bus_initialize(LINK_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK)
        return ResponseStatusContainer::error("spi_bus_initialize failed");

    spi_device_interface_config_t device = {};
    device.mode = 0;
    device.clock_speed_hz = LINK_CLOCK_HZ;
    device.spics_io_num = LINK_CS;
    device.queue_size = 1;
    device.cs_ena_pretrans = 4;    // the slave needs a moment to see the chip select
    device.cs_ena_posttrans = 4;

    err = spi_bus_add_device(LINK_HOST, &device, &_device);
    if (err != ESP_OK)
    {
        spi_bus_free(LINK_HOST);
        return ResponseStatusContainer::error("spi_bus_add_device failed");
    }

    _initialized = true;
    LOG_INFO(TAG, "SPI master ready: MOSI=%d MISO=%d CLK=%d CS=%d at %d Hz, %u byte frames", LINK_MOSI, LINK_MISO,
             LINK_CLK, LINK_CS, LINK_CLOCK_HZ, static_cast<unsigned>(HOST_LINK_FRAME_SIZE));
    return ResponseStatusContainer::ok("Antenna board link ready");
}

bool AntennaBoardTransmitter::exchange(const HostLinkFrame &out)
{
    std::memcpy(_txBuffer, &out, HOST_LINK_FRAME_SIZE);
    std::memset(_rxBuffer, 0, HOST_LINK_FRAME_SIZE);

    spi_transaction_t transaction = {};
    transaction.length = HOST_LINK_FRAME_SIZE * 8;
    transaction.tx_buffer = _txBuffer;
    transaction.rx_buffer = _rxBuffer;

    if (spi_device_transmit(_device, &transaction) != ESP_OK) return false;

    HostLinkFrame in;
    std::memcpy(&in, _rxBuffer, HOST_LINK_FRAME_SIZE);
    if (!hostLinkValid(in)) return false;

    handleIncoming(in);
    return true;
}

void AntennaBoardTransmitter::handleIncoming(const HostLinkFrame &frame)
{
    _exchanges++;
    _lastExchangeMs = nowMs();

    switch (static_cast<HostLinkType>(frame.type))
    {
    case HostLinkType::Status:
        if (frame.length >= sizeof(HostLinkStatus)) std::memcpy(&_status, frame.payload, sizeof(HostLinkStatus));
        break;

    case HostLinkType::Command:
        if (frame.length >= sizeof(CommandPacket))
        {
            std::memcpy(&_command, frame.payload, sizeof(CommandPacket));
            _commandPending = true;
            LOG_INFO(TAG, "Command 0x%02X received from the ground station", _command.command_id);
        }
        break;

    default:
        break;
    }
}

ResponseStatusContainer AntennaBoardTransmitter::transmit(TransmitDataType data)
{
    if (!_initialized) return ResponseStatusContainer::error("Link not initialized");

    const std::vector<uint8_t> *payload = std::get_if<std::vector<uint8_t>>(&data);
    if (payload == nullptr || payload->empty())
        return ResponseStatusContainer::error("Only byte arrays are supported by this link");
    if (payload->size() > HOST_LINK_PAYLOAD_SIZE)
        return ResponseStatusContainer::error("Payload too large");

    HostLinkFrame out;
    hostLinkBuild(out, HostLinkType::Telemetry, payload->data(), static_cast<uint8_t>(payload->size()), _sequence++);

    if (!exchange(out))
        return ResponseStatusContainer::error("No valid answer from the antenna board");

    return ResponseStatusContainer::ok("Packet sent successfully");
}

bool AntennaBoardTransmitter::receive(CommandPacket *packet)
{
    if (!_initialized || packet == nullptr) return false;

    // Every transmission is a full duplex transfer, so a command from the ground
    // comes back with the telemetry that was just sent and is already in.
    if (!_commandPending && COMMAND_POLL_INTERVAL_MS > 0 && nowMs() - _lastExchangeMs >= COMMAND_POLL_INTERVAL_MS)
    {
        HostLinkFrame out;
        hostLinkBuild(out, HostLinkType::Idle, nullptr, 0, _sequence++);
        exchange(out);
    }

    if (!_commandPending) return false;
    *packet = _command;
    _commandPending = false;
    return true;
}

bool AntennaBoardTransmitter::isLinked() const
{
    return _exchanges > 0 && (nowMs() - _lastExchangeMs) < LINK_TIMEOUT_MS;
}

#endif  // CONFIG_TELEMETRY_LINK_ANTENNA_BOARD
