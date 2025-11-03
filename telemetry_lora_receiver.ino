/**
 * @file telemetry_receiver.ino
 * @brief LoRa Telemetry Receiver with Binary Packet Parsing and Display
 * 
 * This sketch receives telemetry packets via LoRa, reassembles multi-packet
 * sequences, deserializes binary TelemetryPacket structure, and displays
 * human-readable data on Serial Monitor and OLED.
 * 
 * Architecture:
 * Flight Computer --[ESP-NOW]--> Transmitter --[LoRa Radio]--> This Receiver
 *                                                                      |
 *                                                     Displays: Serial Monitor + OLED
 * 
 * Features:
 * - Receives Packet structures via LoRa (SX1262/SX1268 module)
 * - Reassembles multi-chunk sequences by messageId
 * - Deserializes binary TelemetryPacket structure (~64 bytes)
 * - Displays detailed sensor data on Serial Monitor
 * - Shows key telemetry metrics on OLED display
 * - Tracks metrics: throughput, packet loss, RSSI/SNR
 * - No ESP-NOW forwarding (display only)
 * 
 * Hardware: 
 * - Heltec WiFi LoRa 32 V3 (ESP32-S3 + SX1262 + OLED)
 * 
 * Configuration:
 * - Configure LoRa parameters to match transmitter
 * - Adjust SEQUENCE_TIMEOUT_MS and display update intervals as needed
 */

#include <heltec_unofficial.h>
#include <RadioLib.h>
#include <map>
#include <vector>
#include <cstring>
// ============================================================================
// CONFIGURATION
// ============================================================================


// LoRa Radio Parameters (must match transmitter)
constexpr float LORA_FREQUENCY = 868.0;      // MHz
constexpr float LORA_BANDWIDTH = 125.0;      // kHz
constexpr uint8_t LORA_SPREADING_FACTOR = 7; // SF7: ~150-200ms/packet, ~5 pkt/s, range ~5-8km (maximum speed)
constexpr uint8_t LORA_CODING_RATE = 7;
constexpr int8_t LORA_OUTPUT_POWER = 14;     // dBm
constexpr uint16_t LORA_PREAMBLE_LENGTH = 8;
constexpr uint8_t LORA_SYNC_WORD = 0x12;

// Timeout for incomplete sequences (ms)
constexpr unsigned long SEQUENCE_TIMEOUT_MS = 2000;

// Interval for printing metrics to Serial (ms)
constexpr unsigned long METRICS_INTERVAL_MS = 10000;

// Interval for updating OLED display (ms)
constexpr unsigned long DISPLAY_UPDATE_INTERVAL_MS = 500;

// Fixed packet size (always 80 bytes on the wire)
constexpr size_t FIXED_PACKET_SIZE = 70;

// Maximum payload size per packet (must match Packet.hpp: LORA_MAX_PAYLOAD_SIZE = 241)
constexpr size_t LORA_MAX_PAYLOAD_SIZE = 57;
// ============================================================================
// PACKET STRUCTURE (must match transmitter - Packet.hpp)
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
    uint8_t data[LORA_MAX_PAYLOAD_SIZE]; // 64 bytes
} __attribute__((packed));

// Full Packet structure (250 bytes total)
struct Packet
{
    PacketHeader header;         // 7 bytes
    PacketPayload payload;       // 241 bytes (accessed via .payload.data[])
    uint16_t crc;                // 2 bytes
    // Total: 7 + 241 + 2 = 250 bytes
} __attribute__((packed));

// Flag bitmasks
constexpr uint8_t FLAG_START = 0x01;
constexpr uint8_t FLAG_END = 0x02;

// ============================================================================
// TELEMETRY PACKET STRUCTURE (must match flight computer)
// ============================================================================

/**
 * @brief Binary telemetry packet structure used by flight computer.
 * 
 * This structure is fragmented by PacketManager into one or more Packet structures.
 * After reassembly, we cast the byte vector back to this structure.
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

// Latest received telemetry (for display)
TelemetryPacket latestTelemetry = {};
bool telemetryValid = false;
unsigned long lastTelemetryTime = 0;

// ============================================================================
// CRC16 CALCULATION (must match transmitter)
// ============================================================================

uint16_t calculateCRC16(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

bool validatePacketCRC(const Packet& packet)
{
    // Calculate CRC over everything except the CRC field itself
    size_t crcDataLength = offsetof(Packet, crc);
    uint16_t calculatedCRC = calculateCRC16(reinterpret_cast<const uint8_t*>(&packet), crcDataLength);
    return calculatedCRC == packet.crc;
}

// ============================================================================
// SEQUENCE TRACKING
// ============================================================================

struct SequenceBuffer
{
    std::map<uint8_t, Packet> chunks;  // chunkIndex -> Packet
    unsigned long lastUpdateTime;
    uint8_t expectedChunks;  // Total number of chunks (determined from flags)
    bool startReceived;
    bool endReceived;
    
    SequenceBuffer() : lastUpdateTime(0), expectedChunks(0), startReceived(false), endReceived(false) {}
    
    bool isComplete() const
    {
        if (!startReceived || !endReceived)
            return false;
        
        // Check if we have all chunks from 0 to expectedChunks-1
        if (chunks.size() != expectedChunks)
            return false;
        
        for (uint8_t i = 0; i < expectedChunks; i++)
        {
            if (chunks.find(i) == chunks.end())
                return false;
        }
        
        return true;
    }
    
    void addChunk(const Packet& packet)
    {
        chunks[packet.header.chunkIndex] = packet;
        lastUpdateTime = millis();
        
        if (packet.header.flags & FLAG_START)
        {
            startReceived = true;
        }
        
        if (packet.header.flags & FLAG_END)
        {
            endReceived = true;
            expectedChunks = packet.header.chunkIndex + 1;
        }
    }
    
    bool isTimedOut(unsigned long currentTime) const
    {
        return (currentTime - lastUpdateTime) > SEQUENCE_TIMEOUT_MS;
    }
    
    std::vector<uint8_t> reassemble() const
    {
        std::vector<uint8_t> message;
        
        for (uint8_t i = 0; i < expectedChunks; i++)
        {
            auto it = chunks.find(i);
            if (it == chunks.end())
            {
                // Missing chunk, should not happen if isComplete() returned true
                return std::vector<uint8_t>();
            }
            
            const Packet& chunk = it->second;
            // Access payload via .payload.data[] to match Packet.hpp structure
            message.insert(message.end(), 
                          chunk.payload.data, 
                          chunk.payload.data + chunk.header.payloadSize);
        }
        
        return message;
    }
};

// Active sequences being reassembled
std::map<uint16_t, SequenceBuffer> activeSequences;

// ============================================================================
// LORA RADIO
// ============================================================================

// Flag to indicate if a packet has been received
volatile bool receivedFlag = false;

// ISR for LoRa packet reception
void IRAM_ATTR setFlag(void)
{
    receivedFlag = true;
}

// ============================================================================
// METRICS TRACKING
// ============================================================================

struct Metrics
{
    unsigned long packetsReceived = 0;
    unsigned long packetsLost = 0;  // CRC failures
    unsigned long sequencesComplete = 0;
    unsigned long sequencesDiscarded = 0;  // Timeout or missing chunks
    unsigned long telemetryPacketsDecoded = 0;
    unsigned long bytesReceived = 0;
    unsigned long lastResetTime = 0;
    
    // Last packet radio metrics
    float lastRSSI = 0.0f;
    float lastSNR = 0.0f;
    
    void reset()
    {
        packetsReceived = 0;
        packetsLost = 0;
        sequencesComplete = 0;
        sequencesDiscarded = 0;
        telemetryPacketsDecoded = 0;
        bytesReceived = 0;
        lastResetTime = millis();
        lastRSSI = 0.0f;
        lastSNR = 0.0f;
    }
    
    float getThroughputKbps() const
    {
        unsigned long elapsed = millis() - lastResetTime;
        if (elapsed == 0)
            return 0.0f;
        
        // Convert bytes to bits, milliseconds to seconds: (bytes * 8) / (ms / 1000) = (bytes * 8000) / ms
        return (bytesReceived * 8000.0f) / elapsed;  // kbps
    }
    
    float getPacketLossRate() const
    {
        unsigned long total = packetsReceived + packetsLost;
        if (total == 0)
            return 0.0f;
        
        return (packetsLost * 100.0f) / total;
    }
} metrics;

// ============================================================================
// TELEMETRY PROCESSING
// ============================================================================

void processTelemetryPacket(const std::vector<uint8_t>& message)
{
    // Validate size
    if (message.size() != sizeof(TelemetryPacket))
    {
        Serial.printf("[TELEMETRY] Invalid size: %u bytes (expected %u)\n", 
                      message.size(), sizeof(TelemetryPacket));
        return;
    }
    
    // Cast to TelemetryPacket structure
    TelemetryPacket telemetry;
    memcpy(&telemetry, message.data(), sizeof(TelemetryPacket));
    
    // Update global latest telemetry
    latestTelemetry = telemetry;
    telemetryValid = true;
    lastTelemetryTime = millis();
    metrics.telemetryPacketsDecoded++;
    
    // Print detailed telemetry to Serial
    Serial.println("\n========== TELEMETRY PACKET ==========");
    Serial.printf("Timestamp:     %lu ms\n", telemetry.timestamp);
    Serial.printf("Data Valid:    %s\n", telemetry.dataValid ? "YES" : "NO");
    
    if (telemetry.dataValid)
    {
        Serial.println("\n--- IMU ---");
        Serial.printf("Accel:  X=%.3f  Y=%.3f  Z=%.3f m/s²\n", 
                      telemetry.imu.accel_x, telemetry.imu.accel_y, telemetry.imu.accel_z);
        float accel_mag = sqrt(telemetry.imu.accel_x * telemetry.imu.accel_x +
                              telemetry.imu.accel_y * telemetry.imu.accel_y +
                              telemetry.imu.accel_z * telemetry.imu.accel_z);
        Serial.printf("Accel Magnitude: %.3f m/s²\n", accel_mag);
        Serial.printf("Gyro:   X=%.3f  Y=%.3f  Z=%.3f rad/s\n", 
                      telemetry.imu.gyro_x, telemetry.imu.gyro_y, telemetry.imu.gyro_z);
        
        Serial.println("\n--- Barometer 1 ---");
        Serial.printf("Pressure:    %.2f hPa\n", telemetry.baro1.pressure);
        Serial.printf("Temperature: %.2f °C\n", telemetry.baro1.temperature);
        // Estimate altitude from pressure (assuming standard atmosphere at sea level)
        float altitude1 = 44330.0f * (1.0f - pow(telemetry.baro1.pressure / 1013.25f, 0.1903f));
        Serial.printf("Altitude:    %.2f m (estimated)\n", altitude1);
        
        Serial.println("\n--- Barometer 2 ---");
        Serial.printf("Pressure:    %.2f hPa\n", telemetry.baro2.pressure);
        Serial.printf("Temperature: %.2f °C\n", telemetry.baro2.temperature);
        float altitude2 = 44330.0f * (1.0f - pow(telemetry.baro2.pressure / 1013.25f, 0.1903f));
        Serial.printf("Altitude:    %.2f m (estimated)\n", altitude2);
        
        Serial.println("\n--- GPS ---");
        Serial.printf("Latitude:    %.6f°\n", telemetry.gps.latitude);
        Serial.printf("Longitude:   %.6f°\n", telemetry.gps.longitude);
        Serial.printf("Altitude:    %.2f m\n", telemetry.gps.altitude);
    }
    
    Serial.printf("\nRadio: RSSI=%.1f dBm | SNR=%.1f dB\n", metrics.lastRSSI, metrics.lastSNR);
    Serial.println("======================================\n");
}

// ============================================================================
// LORA PACKET PROCESSING
// ============================================================================

void processLoRaPacket()
{
    if (!receivedFlag)
        return;
    
    receivedFlag = false;
    
    // Read received data - ALWAYS expect FIXED_PACKET_SIZE bytes
    uint8_t buffer[FIXED_PACKET_SIZE];
    int state = radio.readData(buffer, FIXED_PACKET_SIZE);
    
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.printf("[LORA] Read failed, code: %d\n", state);
        radio.startReceive();  // Restart receive
        return;
    }
    
    // Get packet info
    float rssi = radio.getRSSI();
    float snr = radio.getSNR();
    
    // Check if data length matches expected fixed size
    size_t len = radio.getPacketLength();
    if (len != FIXED_PACKET_SIZE)
    {
        Serial.printf("[RX] Invalid packet size: %d (expected %d)\n", len, FIXED_PACKET_SIZE);
        radio.startReceive();
        return;
    }
    
    // Copy to Packet structure
    Packet packet;
    memcpy(&packet, buffer, sizeof(Packet));
    
    // Validate CRC
    if (!validatePacketCRC(packet))
    {
        metrics.packetsLost++;
        Serial.printf("[RX] CRC validation failed for messageId=%u chunkIndex=%u (RSSI: %.1f dBm, SNR: %.1f dB)\n", 
                      packet.header.messageId, packet.header.chunkIndex, rssi, snr);
        radio.startReceive();
        return;
    }
    
    // Update metrics
    metrics.packetsReceived++;
    metrics.bytesReceived += packet.header.payloadSize;
    metrics.lastRSSI = rssi;
    metrics.lastSNR = snr;
    
    Serial.printf("[RX] Valid packet: msgId=%u chunk=%u/%u flags=0x%02X size=%u (RSSI: %.1f dBm, SNR: %.1f dB)\n",
                  packet.header.messageId, packet.header.chunkIndex, packet.header.totalChunks,
                  packet.header.flags, packet.header.payloadSize, rssi, snr);
    
    // Add to sequence buffer
    uint16_t msgId = packet.header.messageId;
    activeSequences[msgId].addChunk(packet);
    
    // Check if sequence is complete
    if (activeSequences[msgId].isComplete())
    {
        Serial.printf("[REASSEMBLY] Complete sequence for msgId=%u\n", msgId);
        
        // Reassemble message
        std::vector<uint8_t> message = activeSequences[msgId].reassemble();
        
        if (!message.empty())
        {
            metrics.sequencesComplete++;
            
            // Process telemetry packet (deserialize and display)
            processTelemetryPacket(message);
        }
        
        // Remove completed sequence
        activeSequences.erase(msgId);
    }
    
    // Restart receive
    radio.startReceive();
}

// ============================================================================
// SEQUENCE CLEANUP
// ============================================================================

void cleanupTimedOutSequences()
{
    unsigned long currentTime = millis();
    std::vector<uint16_t> toRemove;
    
    for (auto& entry : activeSequences)
    {
        if (entry.second.isTimedOut(currentTime))
        {
            toRemove.push_back(entry.first);
            metrics.sequencesDiscarded++;
            Serial.printf("[TIMEOUT] Discarding incomplete sequence msgId=%u (received %u chunks)\n",
                          entry.first, entry.second.chunks.size());
        }
    }
    
    for (uint16_t msgId : toRemove)
    {
        activeSequences.erase(msgId);
    }
}

// ============================================================================
// METRICS REPORTING
// ============================================================================

void printMetrics()
{
    Serial.println("\n========== RECEIVER METRICS ==========");
    Serial.printf("Packets Received:      %lu\n", metrics.packetsReceived);
    Serial.printf("Packets Lost (CRC):    %lu\n", metrics.packetsLost);
    Serial.printf("Packet Loss Rate:      %.2f%%\n", metrics.getPacketLossRate());
    Serial.printf("Sequences Complete:    %lu\n", metrics.sequencesComplete);
    Serial.printf("Sequences Discarded:   %lu\n", metrics.sequencesDiscarded);
    Serial.printf("Telemetry Decoded:     %lu\n", metrics.telemetryPacketsDecoded);
    Serial.printf("Bytes Received:        %lu\n", metrics.bytesReceived);
    Serial.printf("Throughput:            %.2f kbps\n", metrics.getThroughputKbps());
    Serial.printf("Active Sequences:      %d\n", activeSequences.size());
    Serial.printf("Last RSSI:             %.1f dBm\n", metrics.lastRSSI);
    Serial.printf("Last SNR:              %.1f dB\n", metrics.lastSNR);
    Serial.printf("Heap Free:             %u bytes\n", ESP.getFreeHeap());
    Serial.println("======================================\n");
}

// ============================================================================
// OLED DISPLAY UPDATE
// ============================================================================

void updateDisplay()
{
    display.clear();
    
    // Title bar with packet count
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    char title[32];
    snprintf(title, sizeof(title), "RX:%lu Err:%lu", metrics.packetsReceived, metrics.packetsLost);
    display.drawString(0, 0, title);
    display.drawHorizontalLine(0, 12, 128);
    
    // Check if we have valid telemetry data (not too old)
    if (telemetryValid && (millis() - lastTelemetryTime < 5000))
    {
        char buf[32];
        
        if (latestTelemetry.dataValid)
        {
            // Line 1: Altitude (average of two barometers)
            float alt1 = 44330.0f * (1.0f - pow(latestTelemetry.baro1.pressure / 1013.25f, 0.1903f));
            float alt2 = 44330.0f * (1.0f - pow(latestTelemetry.baro2.pressure / 1013.25f, 0.1903f));
            float avgAlt = (alt1 + alt2) / 2.0f;
            snprintf(buf, sizeof(buf), "Alt:%.1fm", avgAlt);
            display.drawString(0, 15, buf);
            
            // Line 2: Acceleration magnitude
            float accel_mag = sqrt(latestTelemetry.imu.accel_x * latestTelemetry.imu.accel_x +
                                  latestTelemetry.imu.accel_y * latestTelemetry.imu.accel_y +
                                  latestTelemetry.imu.accel_z * latestTelemetry.imu.accel_z);
            snprintf(buf, sizeof(buf), "Acc:%.2fm/s2", accel_mag);
            display.drawString(0, 26, buf);
            
            // Line 3: GPS coordinates
            snprintf(buf, sizeof(buf), "%.4f,%.4f", 
                    latestTelemetry.gps.latitude, latestTelemetry.gps.longitude);
            display.setFont(ArialMT_Plain_10);
            display.drawString(0, 37, buf);
            
            // Line 4: Temperature (average)
            float avgTemp = (latestTelemetry.baro1.temperature + latestTelemetry.baro2.temperature) / 2.0f;
            snprintf(buf, sizeof(buf), "T:%.1fC", avgTemp);
            display.drawString(0, 48, buf);
        }
        else
        {
            display.drawString(0, 25, "Data Invalid");
        }
        
        // Signal quality (bottom right)
        char signal[16];
        snprintf(signal, sizeof(signal), "%.0f/%.0f", metrics.lastRSSI, metrics.lastSNR);
        display.setTextAlignment(TEXT_ALIGN_RIGHT);
        display.drawString(128, 48, signal);
    }
    else
    {
        // No telemetry or stale data
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.drawString(64, 30, "Waiting for");
        display.drawString(64, 42, "telemetry...");
    }
    
    display.display();
}

// ============================================================================
// SETUP
// ============================================================================

void setup()
{
    // Initialize Heltec board (includes Serial, display, and radio)
    heltec_setup();
    
    Serial.println("\n\n===================================================");
    Serial.println("LoRa Telemetry Receiver Starting...");
    Serial.println("===================================================\n");
    
    // Initialize OLED display
    Serial.println("[OLED] Initializing display...");
    display.init();
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "LoRa Receiver");
    display.drawString(0, 15, "Initializing...");
    display.display();
    Serial.println("[OK] OLED display initialized\n");
    
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
    
    // Set up interrupt for LoRa reception
    radio.setDio1Action(setFlag);
    
    // Start listening for LoRa packets
    state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.printf("[ERROR] Failed to start LoRa receive, code: %d\n", state);
        while (true) delay(1000);
    }
    
    Serial.println("[OK] LoRa receiver started\n");
    
    // Initialize metrics
    metrics.reset();
    
    Serial.println("\n[READY] Waiting for LoRa telemetry packets...\n");
    
    // Update display to show ready state
    display.clear();
    display.setFont(ArialMT_Plain_10);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 0, "LoRa Receiver");
    display.drawString(0, 15, "READY");
    display.drawString(0, 30, "Waiting for");
    display.drawString(0, 45, "packets...");
    display.display();
    delay(2000);  // Show ready screen for 2 seconds
}

// ============================================================================
// MAIN LOOP
// ============================================================================

unsigned long lastMetricsPrint = 0;
unsigned long lastCleanup = 0;
unsigned long lastDisplayUpdate = 0;

void loop()
{
    heltec_loop();  // Keep Heltec library happy
    
    unsigned long currentTime = millis();
    
    // Process any received LoRa packets
    processLoRaPacket();
    
    // Periodic metrics printing
    if (currentTime - lastMetricsPrint >= METRICS_INTERVAL_MS)
    {
        printMetrics();
        lastMetricsPrint = currentTime;
    }
    
    // Periodic display update
    if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL_MS)
    {
        updateDisplay();
        lastDisplayUpdate = currentTime;
    }
    
    // Periodic cleanup of timed-out sequences
    if (currentTime - lastCleanup >= 1000)  // Check every second
    {
        cleanupTimedOutSequences();
        lastCleanup = currentTime;
    }
    
    // Small delay to prevent busy-waiting
    delay(10);
}