#include "E220LoRaTransmitter.hpp"
#include <pins.h>
#include <Logger.hpp>

/**
 * @brief Initialize the LoRa module with default configuration.
 *
 * @return ResponseStatusContainer
 */
ResponseStatusContainer E220LoRaTransmitter::init()
{
    // GPIO 41/42 are JTAG pins so they need to be reassigned.
    // For a VERY werid reason, if we don't reset them,
    // only the first configuration of the chip will work.
    LOG_DEBUG("LoRa", "Init. Resetting JTAG pins...");
    if (_m0Pin != GPIO_NUM_NC) gpio_reset_pin(_m0Pin);
    if (_m1Pin != GPIO_NUM_NC) gpio_reset_pin(_m1Pin);

    auto res = transmitter.begin();
    auto description = res ? "LoRa module initialized successfully." : "Failed to initialize LoRa module.";
    return ResponseStatusContainer(res, description);
}

ResponseStatusContainer E220LoRaTransmitter::init(Configuration config)
{
    LOG_DEBUG("LoRa", "Init. Resetting JTAG pins...");
    if (_m0Pin != GPIO_NUM_NC) gpio_reset_pin(_m0Pin);
    if (_m1Pin != GPIO_NUM_NC) gpio_reset_pin(_m1Pin);

    transmitter.begin();
    LOG_INFO("LoRa", "Reconfiguring transmitter...");
    auto result = this->configure(config);

    if (_serial && _rxPin != GPIO_NUM_NC && _txPin != GPIO_NUM_NC) {
        _serial->begin(115200, SERIAL_8N1, _rxPin, _txPin);
    }

    return result;
}

/**
 * @brief Converts the data to a byte array.
 *
 * @param data
 * @return std::vector<uint8_t>
 */
std::vector<uint8_t> convertToByteArray(const TransmitDataType &data)
{
    // Binary: return as-is.
    if (std::holds_alternative<std::vector<uint8_t>>(data))
        return std::get<std::vector<uint8_t>>(data);

    // Text: convert to string and append terminator.
    std::string sData;
    const char terminator = 0x17;
    if (std::holds_alternative<char *>(data))
        sData = std::get<char *>(data);
    else if (std::holds_alternative<String>(data))
        sData = std::get<String>(data).c_str();
    else if (std::holds_alternative<std::string>(data))
        sData = std::get<std::string>(data);
    else if (std::holds_alternative<nlohmann::json>(data))
        sData = std::get<nlohmann::json>(data).dump();
    else
        return {};

    if (sData.back() != terminator)
        sData += terminator;

    return std::vector<uint8_t>(sData.begin(), sData.end());
}

std::vector<uint8_t> compressData(std::vector<uint8_t> data)
{
    // TODO: use delta encoding to compress the data
    return data;
}

ResponseStatusContainer E220LoRaTransmitter::transmit(TransmitDataType data)
{
    std::vector<uint8_t> byteArray = convertToByteArray(data);
    if (byteArray.empty())
    {
        return ResponseStatusContainer(ERR_E220_UNKNOWN, "Couldn't convert data to byte array. Wrong data type or empty buffer.");
    }

    std::vector<uint8_t> compressedData = compressData(byteArray);
    size_t dataLength = compressedData.size();
    size_t offset = 0;

    // Preparazione del pacchetto
    Packet packet = {};
    packet.header.messageId = this->packetNumber++;
    // Calcolo del numero totale di chunk per rientrare nei 199 byte massimi.
    packet.header.totalChunks = (dataLength + LORA_MAX_PAYLOAD_SIZE - 1) / LORA_MAX_PAYLOAD_SIZE;
    packet.header.chunkIndex = 0;
    packet.header.flags = 0;
    
    while (offset < dataLength)
    {
        if (packet.header.chunkIndex == 0)
            packet.header.flags |= 0x01;  // Start of Message
        else if (packet.header.chunkIndex == packet.header.totalChunks - 1)
            packet.header.flags |= 0x02;  // Start of Message

        // packet.header.timestamp = static_cast<uint32_t>(time(nullptr));
        uint8_t payloadSize = std::min(dataLength - offset, static_cast<size_t>(LORA_MAX_PAYLOAD_SIZE));
        
        // Dimensione del payload
        packet.header.payloadSize = payloadSize;
        // packet.header.chunkSize = payloadSize + HEADER_SIZE + CRC_SIZE;
        // Suddivisione del pacchetto in chunk

        memcpy(packet.payload.data, compressedData.data() + offset, payloadSize);

        if (payloadSize < LORA_MAX_PAYLOAD_SIZE)
        {
            // Se il chunk è più piccolo del massimo, riempi con 0x01 dopo il termine del payload
            memset(packet.payload.data + payloadSize, 0x01, LORA_MAX_PAYLOAD_SIZE - payloadSize);
        }

        packet.calculateCRC();
        // Per debug: Stampa tutto il pacchetto in esadecimale
        #ifdef __DEBUG__
        packet.printPacket();
        #endif
        auto rc = this->transmitter.sendFixedMessage(LORA_RECEIVER_ADDH, LORA_RECEIVER_ADDL, LORA_CHANNEL,
                                                     &packet, sizeof(Packet));
        /* Delay per evitare collisioni (perdita di pacchetti),
            empiricamente il minimo è 10ms. */
        vTaskDelay(pdMS_TO_TICKS(10));

        if (rc.code != E220_SUCCESS)
        {
            return ResponseStatusContainer(rc.code, rc.getResponseDescription());
        }
        packet.header.chunkIndex++;
        // Sposta l'offset al prossimo chunk
        offset += payloadSize;
    }

    return ResponseStatusContainer(E220_SUCCESS, "Data sent successfully.");
}

ResponseStatusContainer E220LoRaTransmitter::configure(Configuration configuration)
{
    auto response = transmitter.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
    return ResponseStatusContainer(response.code, response.getResponseDescription());
}

ResponseStructContainer E220LoRaTransmitter::getConfiguration()
{
    return transmitter.getConfiguration();
}

String E220LoRaTransmitter::getConfigurationString(Configuration configuration) const
{
    String result;
    result += "----------------------------------------\n";

    result += "HEAD : " + String(configuration.COMMAND, HEX) + " " + String(configuration.STARTING_ADDRESS, HEX) + " " + String(configuration.LENGHT, HEX) + "\n\n";
    result += "AddH : " + String(configuration.ADDH, HEX) + "\n";
    result += "AddL : " + String(configuration.ADDL, HEX) + "\n\n";
    result += "Chan : " + String(configuration.CHAN, DEC) + " -> " + configuration.getChannelDescription() + "\n\n";
    result += "SpeedParityBit     : " + String(configuration.SPED.uartParity, BIN) + " -> " + configuration.SPED.getUARTParityDescription() + "\n";
    result += "SpeedUARTDatte     : " + String(configuration.SPED.uartBaudRate, BIN) + " -> " + configuration.SPED.getUARTBaudRateDescription() + "\n";
    result += "SpeedAirDataRate   : " + String(configuration.SPED.airDataRate, BIN) + " -> " + configuration.SPED.getAirDataRateDescription() + "\n\n";
    result += "OptionSubPacketSett: " + String(configuration.OPTION.subPacketSetting, BIN) + " -> " + configuration.OPTION.getSubPacketSetting() + "\n";
    result += "OptionTranPower    : " + String(configuration.OPTION.transmissionPower, BIN) + " -> " + configuration.OPTION.getTransmissionPowerDescription() + "\n";
    result += "OptionRSSIAmbientNo: " + String(configuration.OPTION.RSSIAmbientNoise, BIN) + " -> " + configuration.OPTION.getRSSIAmbientNoiseEnable() + "\n\n";
    result += "TransModeWORPeriod : " + String(configuration.TRANSMISSION_MODE.WORPeriod, BIN) + " -> " + configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription() + "\n";
    result += "TransModeEnableLBT : " + String(configuration.TRANSMISSION_MODE.enableLBT, BIN) + " -> " + configuration.TRANSMISSION_MODE.getLBTEnableByteDescription() + "\n";
    result += "TransModeEnableRSSI: " + String(configuration.TRANSMISSION_MODE.enableRSSI, BIN) + " -> " + configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription() + "\n";
    result += "TransModeFixedTrans: " + String(configuration.TRANSMISSION_MODE.fixedTransmission, BIN) + " -> " + configuration.TRANSMISSION_MODE.getFixedTransmissionDescription() + "\n";

    result += "----------------------------------------\n";

    return result;
}

template <typename T>
T E220LoRaTransmitter::getBpsValue(const std::map<int, T> &baudRateMap, T defaultValue) const
{
    auto it = baudRateMap.find(SERIAL_BAUD_RATE);
    if (it != baudRateMap.end())
    {
        return it->second;
    }
    else
    {
        return defaultValue;
    }
}

UART_BPS_RATE E220LoRaTransmitter::getBpsRate() const
{
    static const std::map<int, UART_BPS_RATE> baudRateMap = {
        {1200, UART_BPS_RATE_1200},
        {2400, UART_BPS_RATE_2400},
        {4800, UART_BPS_RATE_4800},
        {9600, UART_BPS_RATE_9600},
        {19200, UART_BPS_RATE_19200},
        {38400, UART_BPS_RATE_38400},
        {57600, UART_BPS_RATE_57600},
        {115200, UART_BPS_RATE_115200}};

    return getBpsValue(baudRateMap, UART_BPS_RATE_9600);
}

UART_BPS_TYPE E220LoRaTransmitter::getBpsType() const
{
    static const std::map<int, UART_BPS_TYPE> baudRateMap = {
        {1200, UART_BPS_1200},
        {2400, UART_BPS_2400},
        {4800, UART_BPS_4800},
        {9600, UART_BPS_9600},
        {19200, UART_BPS_19200},
        {38400, UART_BPS_38400},
        {57600, UART_BPS_57600},
        {115200, UART_BPS_115200}};

    return getBpsValue(baudRateMap, UART_BPS_9600);
}
