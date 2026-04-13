#include "Packet.hpp"
#include <cstdio>

void Packet::calculateCRC()
{
    // CRC-16 algorithm (reflected) with initial value 0xFFFF and polynomial
    // 0xA001 (this is the reflected form of 0x8005 used by CRC-16/ARC).
    uint16_t crc = 0xFFFF;
    const uint8_t *data = reinterpret_cast<const uint8_t *>(this);

    // Compute length up to (but not including) the crc field.
    size_t length = offsetof(Packet, crc);

    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc = crc >> 1;
        }
    }
    this->crc = crc;
}

void Packet::printPacket()
{
    bool som = (this->header.flags & 0x01) != 0;
    bool eom = (this->header.flags & 0x02) != 0;
    bool ackReq = (this->header.flags & 0x04) != 0;

    printf("######## HEADER ########\n");
    printf("Message ID: %u\n", this->header.messageId);
    printf("Flags: 0x%02X (SOM=%d, EOM=%d, ACKReq=%d)\n",
           this->header.flags, som ? 1 : 0, eom ? 1 : 0, ackReq ? 1 : 0);
    printf("Total Chunks: %u\n", this->header.totalChunks);
    printf("Chunk Index (0-based): %u (1-based: %u)\n",
           this->header.chunkIndex, this->header.chunkIndex + 1);
    printf("Payload Size: %u\n", this->header.payloadSize);
    printf("Protocol Version: %u\n", this->header.protocolVersion);
    printf("######## PAYLOAD ########\n");

    // Print only the declared payloadSize bytes (rest may be padding)
    int toPrint = this->header.payloadSize;
    if (toPrint > (int)LORA_MAX_PAYLOAD_SIZE)
        toPrint = LORA_MAX_PAYLOAD_SIZE;
    for (int i = 0; i < toPrint; i++)
    {
        printf("%02X ", this->payload.data[i]);
    }
    printf("\nCRC: 0x%04X\n", this->crc);
}
