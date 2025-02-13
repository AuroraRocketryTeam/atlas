#pragma once

#ifndef MAX_PACKET_SIZE
#define MAX_PACKET_SIZE 199
#endif

#ifndef MAX_PAYLOAD_SIZE // Max packet size (199 bytes) - header size (9 bytes) - crc size (2 bytes)
#define MAX_PAYLOAD_SIZE 188
#endif

#include <cstdint>
#include <cstddef> // Per offsetof

#pragma pack(push, 1) // Evita padding nelle strutture

/**
 * @brief The packet header.
 *
 */
struct PacketHeader
{
    uint8_t packetNumber;
    uint8_t totalChunks;
    uint8_t chunkNumber;
    uint8_t chunkSize;   // In bytes, max 114
    uint8_t payloadSize; // In bytes, max 114
    uint32_t timestamp;  // Unix timestamp
};

/**
 * @brief The payload of a packet.
 *
 */
struct PacketPayload
{
    uint8_t data[MAX_PAYLOAD_SIZE];
};

/**
 * @brief A transmission packet with a header and a payload.
 *
 */
struct Packet
{
    PacketHeader header;
    PacketPayload payload;
    uint16_t crc;
};

#pragma pack(pop) // Ripristina l'allineamento predefinito
/**
 * @brief Calculate the CRC16 of a packet.
 *
 * @param packet the packet (passed by reference)
 * @return uint16_t the CRC16 of the packet
 */
uint16_t calculateCRC(const Packet &packet);