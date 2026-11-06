#pragma once

#include <cstddef>
#include <cstdint>

/**
 * @file HostLinkFrame.hpp
 * @brief Format of packets sent to the Antenna Board over SPI.
 *
 * Manny is the SPI master and transfers whenever it has telemetry to
 * send, and in the same transfer the antenna board answers with a command
 * received from the ground, or with its own status.
 *
 * The frame is a multiple of four bytes so that it can be used directly with the
 * DMA of the SPI. This file has no dependencies and is
 * meant to be copied on both the Antenna Board and Manny firmware.
 */

static constexpr uint16_t HOST_LINK_MAGIC = 0x5541;   ///< 'A', 'U'
static constexpr uint8_t HOST_LINK_VERSION = 1;
static constexpr size_t HOST_LINK_PAYLOAD_SIZE = 96;  ///< one TelemetryPacket, rounded up to keep the frame a multiple of four

/**
 * @brief Content of a frame, in either direction.
 */
enum class HostLinkType : uint8_t
{
    Idle = 0,       ///< nothing to carry
    Telemetry = 1,  ///< Manny to antenna board: the telemetry payload to send
    Command = 2,    ///< antenna board to Manny: command received from the ground
    Status = 3      ///< antenna board to Manny: radio counters and GPS fix
};

#pragma pack(push, 1)
struct HostLinkFrame
{
    uint16_t magic;
    uint8_t version;
    uint8_t type;
    uint8_t length;    ///< used payload bytes
    uint8_t sequence;  ///< incremented by the sender, only for logging
    uint8_t payload[HOST_LINK_PAYLOAD_SIZE];
    uint16_t crc;      ///< CRC for consistency
};

/**
 * @brief Payload of a Status frame.
 */
struct HostLinkStatus
{
    uint32_t uptimeMs;
    uint32_t transmitted;
    uint32_t transmitErrors;
    uint8_t radioReady;
    uint8_t gpsFixType;
    uint8_t gpsSatellites;
    uint8_t gpsCn0Max;
    float latitude;
    float longitude;
    float altitude;
    float groundSpeed;
    uint32_t gpsUtcTime;
};
#pragma pack(pop)

static constexpr size_t HOST_LINK_FRAME_SIZE = sizeof(HostLinkFrame);
// size checks, ensure these are the same as the AntennaBoard!!
static_assert(HOST_LINK_FRAME_SIZE == 104, "HostLinkFrame must stay 104 bytes");
static_assert(HOST_LINK_FRAME_SIZE % 4 == 0, "HostLinkFrame must be a multiple of 4 bytes");
static_assert(sizeof(HostLinkStatus) <= HOST_LINK_PAYLOAD_SIZE, "HostLinkStatus must fit in the payload");

/**
 * @brief CRC-16/MODBUS with initial value 0xFFFF, as used by the radio chip.
 * Repeated here to avoid dependencies in this file.
 */
inline uint16_t hostLinkCrc(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
            crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
    }
    return crc;
}

/**
 * @brief Fill a frame and compute its CRC
 */
inline void hostLinkBuild(HostLinkFrame &frame, HostLinkType type, const void *payload, uint8_t length,
                          uint8_t sequence)
{
    frame = {};
    frame.magic = HOST_LINK_MAGIC;
    frame.version = HOST_LINK_VERSION;
    frame.type = static_cast<uint8_t>(type);
    frame.length = (length > HOST_LINK_PAYLOAD_SIZE) ? HOST_LINK_PAYLOAD_SIZE : length;
    frame.sequence = sequence;
    if (payload != nullptr && frame.length > 0)
    {
        const uint8_t *bytes = static_cast<const uint8_t *>(payload);
        for (uint8_t i = 0; i < frame.length; i++) frame.payload[i] = bytes[i];
    }
    frame.crc = hostLinkCrc(reinterpret_cast<const uint8_t *>(&frame), HOST_LINK_FRAME_SIZE - sizeof(frame.crc));
}

/**
 * @brief Check magic, version and CRC of a received frame
 */
inline bool hostLinkValid(const HostLinkFrame &frame)
{
    if (frame.magic != HOST_LINK_MAGIC || frame.version != HOST_LINK_VERSION) return false;
    if (frame.length > HOST_LINK_PAYLOAD_SIZE) return false;
    const uint16_t crc = hostLinkCrc(reinterpret_cast<const uint8_t *>(&frame), HOST_LINK_FRAME_SIZE - sizeof(frame.crc));
    return crc == frame.crc;
}
