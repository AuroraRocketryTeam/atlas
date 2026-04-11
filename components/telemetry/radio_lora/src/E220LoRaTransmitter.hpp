#pragma once
#include <ITransmitter.hpp>
#include <Packet.hpp>
#include <config.h>
#include <variant>
#include <LoRa_E220.h>
#include <HardwareSerial.h>
#include <cstdint>
#include <driver/gpio.h>

using TransmitDataType = std::variant<char *, String, std::string, nlohmann::json, std::vector<uint8_t>>;

/**
 * @brief Class for the Ebyte E220-900T22D LoRa module transmitter
 *
 */
class E220LoRaTransmitter : public ITransmitter<TransmitDataType>
{
public:
    /**
     * @brief Construct a new E220LoRaTransmitter object.
     *
     * Opens the serial port at 115200. Assumes the module is already configured.
     * Call init(Configuration) if you need to change it.
     *
     * @param serial The serial port for the LoRa module
     * @param txPin  ESP32 TX pin
     * @param rxPin  ESP32 RX pin
     * @param auxPin AUX pin
     * @param m0Pin  M0 pin
     * @param m1Pin  M1 pin
     */
    // E220 TX <-> Our RX
    E220LoRaTransmitter(HardwareSerial &serial, gpio_num_t txPin, gpio_num_t rxPin, gpio_num_t auxPin, gpio_num_t m0Pin, gpio_num_t m1Pin)
        : transmitter(rxPin, txPin, &serial, auxPin, m0Pin, m1Pin, UART_BPS_RATE_115200, SERIAL_8N1),
          _serial(&serial), _txPin(txPin), _rxPin(rxPin), _auxPin(auxPin), _m0Pin(m0Pin), _m1Pin(m1Pin) {};

    ~E220LoRaTransmitter() {
        // Without this, Serial.end() is never called and gpio_set_direction()
        // on those pins (e.g. from the E220 cable test) corrupts the driver state,
        // breaking all subsequent init() call.
        if (_serial) _serial->end();
    }

    /**
     * @brief Construct a new E220LoRaTransmitter object
     *
     * @param serial The serial port for the LoRa module
     * @param auxPin AUX pin
     */
    E220LoRaTransmitter(HardwareSerial &serial, gpio_num_t auxPin)
        : transmitter(&serial, auxPin, GPIO_NUM_NC, GPIO_NUM_NC, UART_BPS_RATE_115200) {};

    /**
     * @brief Construct a new E220LoRaTransmitter object
     *
     * @param serial The serial port for the LoRa module
     */
    explicit E220LoRaTransmitter(HardwareSerial &serial)
        : transmitter(&serial, GPIO_NUM_NC, GPIO_NUM_NC, GPIO_NUM_NC, UART_BPS_RATE_115200) {};

    /**
     * @brief Start the LoRa module.
     *
     * @return ResponseStatusContainer
     */
    ResponseStatusContainer init() override;

    /**
     * @brief Configure the module and start it.
     *
     * @param config The configuration to write (see LoRa_E220.h)
     * @return ResponseStatusContainer
     */
    ResponseStatusContainer init(Configuration config);

    /**
     * @brief Transmit data over LoRa
     *
     * @param data The data to transmit
     * @return ResponseStatusContainer
     */
    ResponseStatusContainer transmit(TransmitDataType data) override;

    /**
     * @brief Set a new configuration for the LoRa module
     *
     * @param configuration The configuration to use
     * @return ResponseStatusContainer
     */
    ResponseStatusContainer configure(Configuration configuration);

    /**
     * @brief Get the configuration object
     *
     * @return ResponseStructContainer
     */
    ResponseStructContainer getConfiguration();

    /**
     * @brief Get the configuration string (an utility function)
     *
     * @param configuration The configuration to convert to a string
     * @return String
     */
    String getConfigurationString(Configuration configuration) const;

private:
    LoRa_E220 transmitter;
    HardwareSerial* _serial = nullptr;
    gpio_num_t _txPin  = GPIO_NUM_NC;
    gpio_num_t _rxPin  = GPIO_NUM_NC;
    gpio_num_t _auxPin = GPIO_NUM_NC;
    gpio_num_t _m0Pin  = GPIO_NUM_NC;
    gpio_num_t _m1Pin  = GPIO_NUM_NC;

    UART_BPS_RATE getBpsRate() const;
    UART_BPS_TYPE getBpsType() const;

    template <typename T>
    T getBpsValue(const std::map<int, T> &baudRateMap, T defaultValue) const;
    uint16_t packetNumber = 1;
};