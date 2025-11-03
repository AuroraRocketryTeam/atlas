/**
 * @file telemetry_lora_transmitter.ino
 * @brief ESP-NOW to LoRa Telemetry Transmitter Bridge
 * 
 * This sketch receives telemetry packets from the flight computer via ESP-NOW
 * and forwards them via LoRa radio to the ground receiver.
 * 
 * Architecture:
 * Flight Computer --[ESP-NOW]--> This Module --[LoRa Radio]--> LoRa Receiver --[ESP-NOW]--> Ground Station
 *                                      ^
 *                                This sketch runs here
 * 
 * Data Flow:
 * 1. Flight computer creates binary TelemetryPacket (~64 bytes) with sensor data
 * 2. PacketManager fragments it into Packet structures (250 bytes each)
 * 3. This module receives Packet structures via ESP-NOW
 * 4. This module forwards Packet structures via LoRa (no reassembly needed)
 * 5. Ground receiver reassembles Packet structures back to TelemetryPacket
 * 
 * Features:
 * - Receives Packet structures via ESP-NOW from flight computer
 * - Forwards packets via LoRa (SX1262/SX1268 module)
 * - Tracks transmission statistics
 * - Periodic Serial logging of metrics
 * - Non-blocking operation
 * - Queue for handling burst transmissions
 * 
 * Hardware: 
 * - Heltec WiFi LoRa 32 V3 (or compatible ESP32 + SX1262/SX1268)
 * 
 * Configuration:
 * - Set FLIGHT_COMPUTER_MAC to the MAC address of the flight computer
 * - Configure LoRa parameters to match receiver
 * - Adjust transmit power and other radio parameters as needed
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <RadioLib.h>

// instead of defining RADIO_BOARD_AUTO,
// board can be specified manually
#define RADIO_BOARD_WIFI_LORA32_V3

// now include RadioBoards
// this must be included AFTER RadioLib!
#include <RadioBoards.h>

// create a new RadioLib module
// "Radio" will be set to the correct module type,
// and "RadioModule" will have the correct pins
Radio radio = new RadioModule();

// ============================================================================
// CONFIGURATION
// ============================================================================

// MAC address of the flight computer 
uint8_t FLIGHT_COMPUTER_MAC[] = {0x74, 0x4D, 0xBD, 0xA0, 0x50, 0x6C};

// LoRa Radio Parameters (must match receiver)
constexpr float LORA_FREQUENCY = 868.0;      // MHz
constexpr float LORA_BANDWIDTH = 125.0;      // kHz
constexpr uint8_t LORA_SPREADING_FACTOR = 7; // SF7: ~150-200ms/packet, ~5 pkt/s, range ~5-8km (maximum speed)
constexpr uint8_t LORA_CODING_RATE = 7;
constexpr int8_t LORA_OUTPUT_POWER = 14;     // dBm
constexpr uint16_t LORA_PREAMBLE_LENGTH = 8;
constexpr uint8_t LORA_SYNC_WORD = 0x12;

// WiFi channel for ESP-NOW (must match flight computer)
constexpr int WIFI_CHANNEL = 1;

// Interval for printing metrics to Serial (ms)
constexpr unsigned long METRICS_INTERVAL_MS = 5000;

// Fixed packet size (always 80 bytes on the wire)
constexpr size_t FIXED_PACKET_SIZE = 70;

// Maximum payload size per packet (must match Packet.hpp: LORA_MAX_PAYLOAD_SIZE = 64)
constexpr size_t LORA_MAX_PAYLOAD_SIZE = 57;

// ============================================================================
// PACKET STRUCTURE (must match flight computer - Packet.hpp)
// ============================================================================

// PacketHeader structure (7 bytes)
struct PacketHeader
{
    uint16_t messageId;          // 2 bytes
    uint8_t totalChunks;         // 1 byte
    uint8_t chunkIndex;          // 1 byte
    uint8_t payloadSize;         // 1 byte
    uint8_t flags;               // 1 byte
    uint8_t protocolVersion;     // 1 byte
} __attribute__((packed));

// PacketPayload structure (241 bytes) - wrapped in struct to match Packet.hpp
struct PacketPayload
{
    uint8_t data[LORA_MAX_PAYLOAD_SIZE]; // 71 bytes
} __attribute__((packed));

// Full Packet structure (250 bytes total)
struct Packet
{
    PacketHeader header;         // 7 bytes
    PacketPayload payload;       // 71 bytes (accessed via .payload.data[])
    uint16_t crc;                // 2 bytes
    // Total: 7 + 241 + 2 = 250 bytes
} __attribute__((packed));

// Flag bitmasks
constexpr uint8_t FLAG_START = 0x01;
constexpr uint8_t FLAG_END = 0x02;

// ============================================================================
// TELEMETRY PACKET STRUCTURE (for reference - actual payload inside Packet)
// ============================================================================

/**
 * @brief Binary telemetry packet structure used by flight computer.
 * 
 * This structure is fragmented by PacketManager into one or more Packet structures.
 * The transmitter bridge doesn't need to parse this - it just forwards Packet structures.
 * This definition is provided for reference and debugging purposes.
 * 
 * Total size: ~64 bytes (fits in single Packet structure)
 */
#pragma pack(push, 1)
struct TelemetryPacket {
    uint32_t timestamp;     ///< Milliseconds since boot
    bool dataValid;         ///< True if sensor data was successfully collected
    
    struct {
        float accel_x, accel_y, accel_z;  ///< Accelerometer (m/s²)
        float gyro_x, gyro_y, gyro_z;     ///< Gyroscope (rad/s)
    } imu;
    
    struct {
        float pressure;     ///< Pressure (hPa)
        float temperature;  ///< Temperature (°C)
    } baro1;
    
    struct {
        float pressure;     ///< Pressure (hPa)
        float temperature;  ///< Temperature (°C)
    } baro2;
    
    struct {
        float latitude;     ///< Latitude (degrees)
        float longitude;    ///< Longitude (degrees)
        float altitude;     ///< GPS altitude (meters)
    } gps;
};
#pragma pack(pop)

// ============================================================================
// METRICS TRACKING
// ============================================================================

struct Metrics
{
    unsigned long packetsReceived = 0;      // Received via ESP-NOW
    unsigned long packetsTransmitted = 0;   // Transmitted via LoRa
    unsigned long packetsFailed = 0;        // Failed LoRa transmissions
    unsigned long bytesReceived = 0;
    unsigned long bytesTransmitted = 0;
    unsigned long lastResetTime = 0;
    
    void reset()
    {
        packetsReceived = 0;
        packetsTransmitted = 0;
        packetsFailed = 0;
        bytesReceived = 0;
        bytesTransmitted = 0;
        lastResetTime = millis();
    }
    
    float getThroughputKbps() const
    {
        unsigned long elapsed = millis() - lastResetTime;
        if (elapsed == 0)
            return 0.0f;
        
        // Convert bytes to bits, milliseconds to seconds: (bytes * 8) / (ms / 1000) = (bytes * 8000) / ms
        return (bytesTransmitted * 8000.0f) / elapsed;  // kbps
    }
    
    float getSuccessRate() const
    {
        unsigned long total = packetsTransmitted + packetsFailed;
        if (total == 0)
            return 100.0f;
        
        return (packetsTransmitted * 100.0f) / total;
    }
} metrics;

// ============================================================================
// PACKET QUEUE
// ============================================================================

// Simple circular buffer for packet queue (stores raw 250-byte packets)
// Increased size to handle burst transmissions
constexpr size_t QUEUE_SIZE = 50;
uint8_t packetQueue[QUEUE_SIZE][FIXED_PACKET_SIZE];
volatile size_t queueHead = 0;
volatile size_t queueTail = 0;
volatile size_t queueCount = 0;

// LoRa transmission state
volatile bool transmitFlag = false;
volatile bool transmitting = false;

bool enqueuePacket(const uint8_t* packetData)
{
    if (queueCount >= QUEUE_SIZE)
    {
        Serial.println("[QUEUE] Queue full, dropping packet!");
        return false;
    }
    
    memcpy(packetQueue[queueHead], packetData, FIXED_PACKET_SIZE);
    queueHead = (queueHead + 1) % QUEUE_SIZE;
    queueCount++;
    
    return true;
}

bool dequeuePacket(uint8_t* packetData)
{
    if (queueCount == 0)
        return false;
    
    memcpy(packetData, packetQueue[queueTail], FIXED_PACKET_SIZE);
    queueTail = (queueTail + 1) % QUEUE_SIZE;
    queueCount--;
    
    return true;
}

// ============================================================================
// ESP-NOW CALLBACKS
// ============================================================================

void onDataReceived(const esp_now_recv_info_t* info, const uint8_t* data, int len)
{
    // Check if data length matches expected fixed packet size
    if (len != FIXED_PACKET_SIZE)
    {
        Serial.printf("[RX] Invalid packet size: %d (expected %d)\n", len, FIXED_PACKET_SIZE);
        return;
    }
    
    // Parse header to extract useful info for logging
    PacketHeader header;
    memcpy(&header, data, sizeof(PacketHeader));
    
    // Update metrics
    metrics.packetsReceived++;
    metrics.bytesReceived += header.payloadSize;
    
    Serial.printf("[RX] Packet from FC: msgId=%u chunk=%u/%u flags=0x%02X size=%u\n",
                  header.messageId, header.chunkIndex, header.totalChunks, 
                  header.flags, header.payloadSize);
    
    // Queue the complete 250-byte packet for LoRa transmission
    if (!enqueuePacket(data))
    {
        Serial.println("[ERROR] Failed to queue packet!");
    }
}

// ============================================================================
// LORA TRANSMISSION
// ============================================================================

// ISR for LoRa transmission complete
void IRAM_ATTR onTransmitDone(void)
{
    transmitFlag = true;
}

void processLoRaTransmission()
{
    // Check if we're currently transmitting
    if (transmitting)
    {
        // Check if transmission completed
        if (transmitFlag)
        {
            transmitFlag = false;
            transmitting = false;
            
            // Transmission completed successfully (interrupt was called)
            // We can't easily check the actual result with startTransmit, 
            // so we assume success if the interrupt fired
            metrics.packetsTransmitted++;
            metrics.bytesTransmitted += FIXED_PACKET_SIZE;
            Serial.printf("[TX] Success (queue: %u)\n", queueCount);
        }
        return; // Still transmitting, come back later
    }
    
    // Check if there's a packet to transmit
    if (queueCount == 0)
        return;
    
    uint8_t packetData[FIXED_PACKET_SIZE];
    if (!dequeuePacket(packetData))
        return;
    
    // Parse header for logging
    PacketHeader header;
    memcpy(&header, packetData, sizeof(PacketHeader));
    
    // Start non-blocking transmission - ALWAYS send FIXED_PACKET_SIZE bytes
    Serial.printf("[TX] Starting transmission: msgId=%u chunk=%u/%u size=%u (queue: %u)\n",
                  header.messageId, header.chunkIndex, header.totalChunks, 
                  header.payloadSize, queueCount);
    
    int state = radio.startTransmit(packetData, FIXED_PACKET_SIZE);
    
    if (state == RADIOLIB_ERR_NONE)
    {
        transmitting = true;
        transmitFlag = false;
    }
    else
    {
        metrics.packetsFailed++;
        Serial.printf("[TX] Start failed, code: %d\n", state);
    }
}

// ============================================================================
// METRICS REPORTING
// ============================================================================

void printMetrics()
{
    Serial.println("\n========== TELEMETRY TRANSMITTER METRICS ==========");
    Serial.printf("Packets Received (ESP-NOW): %lu\n", metrics.packetsReceived);
    Serial.printf("Packets Transmitted (LoRa): %lu\n", metrics.packetsTransmitted);
    Serial.printf("Packets Failed (LoRa):      %lu\n", metrics.packetsFailed);
    Serial.printf("Success Rate:               %.2f%%\n", metrics.getSuccessRate());
    Serial.printf("Bytes Received:             %lu\n", metrics.bytesReceived);
    Serial.printf("Bytes Transmitted:          %lu\n", metrics.bytesTransmitted);
    Serial.printf("Throughput:                 %.2f kbps\n", metrics.getThroughputKbps());
    Serial.printf("Queue Size:                 %u/%u\n", queueCount, QUEUE_SIZE);
    Serial.printf("Heap Free:                  %u bytes\n", ESP.getFreeHeap());
    Serial.println("===================================================\n");
}

// ============================================================================
// SETUP
// ============================================================================

void setup()
{
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n\n===================================================");
    Serial.println("ESP-NOW to LoRa Telemetry Transmitter Starting...");
    Serial.println("===================================================\n");
    
    // Initialize LoRa radio
    Serial.println("[LORA] Initializing radio...");
    int state = radio.begin(LORA_FREQUENCY, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, 
                            LORA_CODING_RATE, LORA_SYNC_WORD, LORA_OUTPUT_POWER, 
                            LORA_PREAMBLE_LENGTH);
    
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.printf("[ERROR] LoRa init failed, code: %d\n", state);
        while (true) delay(1000);
    }
    
    Serial.println("[OK] LoRa radio initialized");
    Serial.printf("     Frequency: %.1f MHz\n", LORA_FREQUENCY);
    Serial.printf("     Bandwidth: %.1f kHz\n", LORA_BANDWIDTH);
    Serial.printf("     Spreading Factor: %d\n", LORA_SPREADING_FACTOR);
    Serial.printf("     Coding Rate: 4/%d\n", LORA_CODING_RATE);
    Serial.printf("     Output Power: %d dBm\n", LORA_OUTPUT_POWER);
    Serial.printf("     Packet Size: %d bytes (fixed)\n", FIXED_PACKET_SIZE);
    Serial.println();
    
    // Set up interrupt for LoRa transmission complete
    radio.setDio1Action(onTransmitDone);
    
    // Set WiFi mode to STA for ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // Print MAC address
    Serial.print("[WIFI] Transmitter MAC Address: ");
    Serial.println(WiFi.macAddress());
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
        Serial.println("[ERROR] ESP-NOW initialization failed!");
        while (true) delay(1000);
    }
    
    Serial.println("[OK] ESP-NOW initialized");
    
    // Register receive callback
    esp_now_register_recv_cb(onDataReceived);
    
    // Add flight computer as peer (for receiving)
    esp_now_peer_info_t fcPeer = {};
    memcpy(fcPeer.peer_addr, FLIGHT_COMPUTER_MAC, 6);
    fcPeer.channel = WIFI_CHANNEL;
    fcPeer.encrypt = false;
    
    if (esp_now_add_peer(&fcPeer) != ESP_OK)
    {
        Serial.println("[ERROR] Failed to add flight computer as peer!");
    }
    else
    {
        Serial.print("[OK] Flight computer peer added: ");
        for (int i = 0; i < 6; i++)
        {
            Serial.printf("%02X%s", FLIGHT_COMPUTER_MAC[i], i < 5 ? ":" : "\n");
        }
    }
    
    // Initialize metrics
    metrics.reset();
    
    Serial.println("\n[INFO] Data Flow:");
    Serial.printf("       Flight Computer creates TelemetryPacket (%d bytes)\n", sizeof(TelemetryPacket));
    Serial.printf("       -> PacketManager fragments to Packet structures (%d bytes each)\n", FIXED_PACKET_SIZE);
    Serial.println("       -> This module forwards Packet structures via LoRa");
    Serial.println("       -> Ground receiver reassembles back to TelemetryPacket");
    
    Serial.println("\n[READY] Waiting for packets from flight computer...\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================

unsigned long lastMetricsPrint = 0;

void loop()
{
    unsigned long currentTime = millis();
    
    // Process any queued LoRa transmissions
    processLoRaTransmission();
    
    // Periodic metrics printing
    if (currentTime - lastMetricsPrint >= METRICS_INTERVAL_MS)
    {
        printMetrics();
        lastMetricsPrint = currentTime;
    }
    
    // Very small delay to prevent busy-waiting but keep loop responsive
    delayMicroseconds(100);
}
