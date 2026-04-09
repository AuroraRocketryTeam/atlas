#pragma once

#include <RadioLib.h>
#include <nlohmann/json.hpp>
#include <variant>
#include <string>
#include <cstdio>
#include <ITransmitter.hpp>
#include <ResponseStatusContainer.hpp>

using TransmitDataType = std::variant<char *, String, std::string, nlohmann::json>;

/**
 * @class LoRaTransmitter
 * @brief Transmitter based on RadioLib for SX1261 module.
 *
 * Manages initialization, basic configuration (frequency, power, BW, SF, CR)
 * and transmission of payloads in various formats (char*, String, std::string, JSON).
 * Default pin mapping: NSS=19, DIO1=2, NRST=14, BUSY=4.
 */
class LoRaTransmitter : public ITransmitter<TransmitDataType>
{
private:
    // Pin configuration per SX1261
    static constexpr int LORA_NSS_PIN = 19;    // Chip Select
    static constexpr int LORA_DIO1_PIN = 2;  // DIO1 (interrupt)
    static constexpr int LORA_NRST_PIN = 14;  // Reset
    static constexpr int LORA_BUSY_PIN = 4;   // Busy
    
    SX1261 radio;
    bool initialized;
    uint16_t packetCounter;
    
    // Configurazione LoRa (modificabile se necessario)
    float frequency = 868.0;        // MHz (Europa)
    int8_t power = 14;              // dBm
    float bandwidth = 125.0;        // kHz
    uint8_t spreadingFactor = 7;    // SF7-SF12
    uint8_t codingRate = 5;         // 4/5
    uint8_t syncWord = 0x12;        // Private network
    
    /**
     * @brief Converts the variant to a string.
     * @param data Data to convert (char*, String, std::string, nlohmann::json).
     * @return std::string Textual representation of the payload.
     */
    std::string dataToString(const TransmitDataType& data) {
        std::string result;
        
        std::visit([&result](const auto& value) {
            using T = std::decay_t<decltype(value)>;
            
            if constexpr (std::is_same_v<T, char*>) {
                result = std::string(value);
            }
            else if constexpr (std::is_same_v<T, String>) {
                result = std::string(value.c_str());
            }
            else if constexpr (std::is_same_v<T, std::string>) {
                result = value;
            }
            else if constexpr (std::is_same_v<T, nlohmann::json>) {
                result = value.dump(); // Serializza JSON in stringa
            }
        }, data);
        
        return result;
    }

public:
    /**
     * @brief Constructor.
     */
    LoRaTransmitter() 
        : radio(new Module(LORA_NSS_PIN, LORA_DIO1_PIN, LORA_NRST_PIN, LORA_BUSY_PIN))
        , initialized(false)
        , packetCounter(0) 
    {
    }
    
    /**
     * @brief Initializes the LoRa SX1261 module.
     * @return ResponseStatusContainer with the initialization status.
     */
    ResponseStatusContainer init() override {
        printf("Initializing SX1261... ");

        // Initialize the module
        int state = radio.begin();
        if (state != RADIOLIB_ERR_NONE) {
            printf("failed, code: %d\n", state);
            return ResponseStatusContainer(state, String("Errore inizializzazione SX1261, codice: ") + String(state));
        }

        if (radio.setFrequency(frequency) != RADIOLIB_ERR_NONE) {
            return ResponseStatusContainer(01, "Errore impostazione frequenza");
        }
        printf("Frequenza: %.1f MHz\n", frequency);

        if (radio.setOutputPower(power) != RADIOLIB_ERR_NONE) {
            return ResponseStatusContainer(02, "Errore impostazione potenza");
        }
        printf("Potenza: %d dBm\n", power);

        if (radio.setBandwidth(bandwidth) != RADIOLIB_ERR_NONE) {
            return ResponseStatusContainer(03, "Errore impostazione bandwidth");
        }
        printf("Bandwidth: %.1f kHz\n", bandwidth);

        if (radio.setSpreadingFactor(spreadingFactor) != RADIOLIB_ERR_NONE) {
            return ResponseStatusContainer(04, "Errore impostazione spreading factor");
        }
        printf("Spreading Factor: SF%u\n", spreadingFactor);

        if (radio.setCodingRate(codingRate) != RADIOLIB_ERR_NONE) {
            return ResponseStatusContainer(05, "Errore impostazione coding rate");
        }
        printf("Coding Rate: 4/%u\n", codingRate);

        if (radio.setSyncWord(syncWord) != RADIOLIB_ERR_NONE) {
            return ResponseStatusContainer(06, "Errore impostazione sync word");
        }
        printf("Sync Word: 0x%02X\n", syncWord);

        initialized = true;
        printf("LoRa SX1261 inizializzato correttamente!\n");
        
        return ResponseStatusContainer(00, "SX1261 inizializzato correttamente");
    }
    
    /**
     * @brief Transmits data through LoRa.
     * @param data The data to transmit (variant type)
     * @return ResponseStatusContainer with the transmission status
     */
    ResponseStatusContainer transmit(TransmitDataType data) override {
        if (!initialized) {
            return ResponseStatusContainer(99, "LoRa not initialized");
        }
        
        // Converts the data to a string
        std::string dataStr = dataToString(data);
        
        // Crea un pacchetto con header per identificazione
        nlohmann::json packet = {
            {"id", packetCounter++},
            {"timestamp", millis()},
            {"payload", dataStr}
        };

        std::string packetStr = packet.dump();

        // Mostra solo i primi 100 caratteri per evitare spam
        if (packetStr.length() > 100) {
            printf("Lora transmission (%zu bytes): %.100s...\n", packetStr.length(), packetStr.c_str());
        } else {
            printf("Lora transmission (%zu bytes): %s\n", packetStr.length(), packetStr.c_str());
        }

        // Trasmetti
        int state = radio.transmit(packetStr.c_str());

        if (state == RADIOLIB_ERR_NONE) {
            printf("Lora transmission completed!\n");
            return ResponseStatusContainer(state, String("Transmission completed, ID: ") + String(packetCounter - 1));
        } else {
            printf("Lora transmission error, code: %d\n", state);
            return ResponseStatusContainer(state, String("Transmission error, code: ") + String(state));
        }
    }
    
    /**
     * @brief Transmits only essential data to reduce payload
     * @param data Complete JSON of the rocket logger
     * @return ResponseStatusContainer with the transmission status
     */
    ResponseStatusContainer transmitCompact(const nlohmann::json& data) {
        if (!initialized) {
            return ResponseStatusContainer(99, "LoRa not initialized");
        }
        
        // Estrae solo i dati critici per ridurre la dimensione del pacchetto
        nlohmann::json compactData = {
            {"id", packetCounter++},
            {"t", millis()}, // timestamp abbreviato
        };
        
        // Cerca i dati BNO055 (IMU principale)
        for (const auto& sensor : data) {
            if (sensor.contains("content") && sensor["content"].contains("source")) {
                std::string source = sensor["content"]["source"];
                
                if (source == "BNO055") {
                    const auto& sensorData = sensor["content"]["sensorData"];
                    if (sensorData.contains("accelerometer")) {
                        compactData["acc"] = {
                            sensorData["accelerometer"]["x"],
                            sensorData["accelerometer"]["y"],
                            sensorData["accelerometer"]["z"]
                        };
                    }
                    if (sensorData.contains("angular_velocity")) {
                        compactData["gyr"] = {
                            sensorData["angular_velocity"]["x"],
                            sensorData["angular_velocity"]["y"],
                            sensorData["angular_velocity"]["z"]
                        };
                    }
                    if (sensorData.contains("board_temperature")) {
                        compactData["temp"] = sensorData["board_temperature"];
                    }
                }
                else if (source == "BAR1") {
                    const auto& sensorData = sensor["content"]["sensorData"];
                    if (sensorData.contains("pressure")) {
                        compactData["pres"] = sensorData["pressure"];
                    }
                }
                else if (source == "Voltage") {
                    const auto& sensorData = sensor["content"]["sensorData"];
                    if (sensorData.contains("Voltage")) {
                        compactData["bat"] = sensorData["Voltage"];
                    }
                }
                else if (source == "MainActuators") {
                    const auto& sensorData = sensor["content"]["sensorData"];
                    if (sensorData.contains("State")) {
                        compactData["main"] = (sensorData["State"] == "ON") ? 1 : 0;
                    }
                }
                else if (source == "DrogueActuators") {
                    const auto& sensorData = sensor["content"]["sensorData"];
                    if (sensorData.contains("State")) {
                        compactData["drogue"] = (sensorData["State"] == "ON") ? 1 : 0;
                    }
                }
            }
        }
        
        std::string compactStr = compactData.dump();

        printf("Compact transmission (%zu bytes): %s\n", compactStr.length(), compactStr.c_str());

        // Trasmetti
        int state = radio.transmit(compactStr.c_str());

        if (state == RADIOLIB_ERR_NONE) {
            printf("Compact transmission completed!\n");
            return ResponseStatusContainer(state, "Compact transmission completed");
        } else {
            printf("Compact transmission error, code: %d\n", state);
            return ResponseStatusContainer(state, String("Compact transmission error, code: ") + String(state));
        }
    }
    
    /**
     * @brief Configures the LoRa parameters.
     * @param freq Frequency in MHz.
     * @param pwr Power in dBm.
     * @param bw Bandwidth in kHz.
     * @param sf Spreading Factor (7..12).
     * @param cr Coding Rate (4/CR).
     */
    void configure(float freq, int8_t pwr, float bw, uint8_t sf, uint8_t cr) {
        frequency = freq;
        power = pwr;
        bandwidth = bw;
        spreadingFactor = sf;
        codingRate = cr;
    }
    
    /**
     * @brief Gets the number of transmitted packets.
     * @return uint16_t Packet counter.
     */
    uint16_t getPacketCount() const {
        return packetCounter;
    }
    
    /**
     * @brief Stampa le statistiche del modulo.
     */
    void printStats() {
        if (!initialized) {
            printf("LoRa non inizializzato\n");
            return;
        }
        printf("=== LoRa SX1261 Statistics ===\n");
    }
};