#include <Wire.h>
#include <HardwareSerial.h>
#include "sensors/ISensor.hpp"
#include "utils/logger/ILogger.hpp"
#include "sensors/BME680/BME680Sensor.hpp"
#include "sensors/BNO055/BNO055Sensor.hpp"
#include "sensors/MPRLS/MPRLSSensor.hpp"
#include "utils/logger/rocket_logger/RocketLogger.hpp"
#include "global/config.h"
#include "telemetry/LoRa/E220LoRaTransmitter.hpp"

ILogger *rocketLogger;
// ISensor *bme680;
ISensor *bno055;
ISensor *mprls;
ITransmitter *loraTransmitter;
HardwareSerial lora_serial(1);

void setup()
{
    lora_serial.begin(SERIAL_BAUD_RATE, SERIAL_8N1, 2, 3);
    Serial.begin(SERIAL_BAUD_RATE);
    delay(500);
    rocketLogger = new RocketLogger();
    // bme680 = new BME680Sensor(BME680_I2C_ADDR_1);
    mprls = new MPRLSSensor();
    bno055 = new BNO055Sensor();
    loraTransmitter = new E220LoRaTransmitter(lora_serial, 4, -1, -1);
    auto transmitterStatus = loraTransmitter->init();
    if (transmitterStatus.getCode() == RESPONSE_STATUS::E220_SUCCESS)
    {
        rocketLogger->logInfo(
            ("LoRa transmitter initialized with configuration: " +
             static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfigurationString(*(Configuration *)(static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfiguration().data)))
                .c_str());
    }
    else
    {
        rocketLogger->logError(
            ("Failed to initialize LoRa transmitter with error: " +
             transmitterStatus.getDescription() +
             " (" + String(transmitterStatus.getCode()) + ")")
                .c_str());
        rocketLogger->logInfo(("Current configuration: " +
                               static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfigurationString(*(Configuration *)(static_cast<E220LoRaTransmitter *>(loraTransmitter)->getConfiguration().data)))
                                  .c_str());
    }
    rocketLogger->logInfo("Setup started.");

    // Define a struct to store sensor initialization information
    struct SensorInitInfo
    {
        ISensor *sensor;
        std::string name;
        std::optional<int> address;
    };

    // Utility vector to initialize all sensors in a loop
    std::vector<SensorInitInfo> sensors = {
        // {bme680, "BME680", BME680_I2C_ADDR_1},
        {mprls, "MPRLS", 0x18},
        {bno055, "BNO055", BNO055_I2C_ADDR}};

    // Lambda function to log sensor initialization result based on initialization success
    auto logInitializationResult = [&](const SensorInitInfo &sensorInfo, bool success)
    {
        if (success)
        {
            rocketLogger->logInfo(sensorInfo.name + " sensor initialized" +
                                  (sensorInfo.address.has_value() ? " on address " + std::to_string(sensorInfo.address.value()) : ""));
        }
        else
        {
            rocketLogger->logError("Failed to initialize " + sensorInfo.name + " sensor" +
                                   (sensorInfo.address.has_value() ? " on address " + std::to_string(sensorInfo.address.value()) : ""));
        }
    };

    // Initialize all sensors
    for (const auto &sensorInfo : sensors)
    {
        bool initSuccess = sensorInfo.sensor->init();

        logInitializationResult(sensorInfo, initSuccess);
    }

    rocketLogger->logInfo("Setup complete.");
    Serial.write(rocketLogger->getJSONAll().dump(4).c_str());
}

void loop()
{
    auto mprlsValue = mprls->getData();
    if (mprlsValue.has_value())
    {
        rocketLogger->logSensorData(mprlsValue.value());
    }
    auto bno055Value = bno055->getData();
    if (bno055Value.has_value())
    {
        rocketLogger->logSensorData(bno055Value.value());
    }
    auto response = loraTransmitter->transmit(rocketLogger->getJSONAll());
    if (response.getCode() != RESPONSE_STATUS::E220_SUCCESS)
    {
        rocketLogger->logError(("Failed to transmit data with error: " + response.getDescription() + " (" + String(response.getCode()) + ")").c_str());
    }
    else if (response.getCode() == RESPONSE_STATUS::ERR_E220_PACKET_TOO_BIG)
    {
        rocketLogger->logError("Data packet too big to transmit.");
        // Split the JSON data into smaller packets by selecting JSON objects one by one and transmit them
        for(auto &data : rocketLogger->getJSONAll())
        {
            auto response = loraTransmitter->transmit(data);
            if (response.getCode() != RESPONSE_STATUS::E220_SUCCESS)
            {
                rocketLogger->logError(("Failed to transmit data with error: " + response.getDescription() + " (" + String(response.getCode()) + ")").c_str());
            } else if (response.getCode() == RESPONSE_STATUS::ERR_E220_PACKET_TOO_BIG)
            {
                rocketLogger->logError("Data packet too big to transmit.");
                // Log the json object's content element by element
                for (json::iterator it = data.begin(); it != data.end(); ++it)
                {
                    rocketLogger->logInfo(it.key() + ": " + it.value().dump());
                }
            }
        }
    }
    else
    {
        rocketLogger->logInfo("Data transmitted successfully.");
    }
    Serial.write(rocketLogger->getJSONAll().dump(4).c_str());
    rocketLogger->clearData();
    delay(2000);
}
