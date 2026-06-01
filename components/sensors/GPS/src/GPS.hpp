#pragma once

#include "ISensor.hpp"
#include "GPSData.hpp"
#include "config.h"
#include <memory>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nmea_parser.h"

/**
 * @class GPS
 * @brief Driver for GNSS modules over UART using ESP-IDF NMEA Parser.
 *
 * This class implements the ISensor interface. It initializes an asynchronous
 * background task to parse standard NMEA sentences (GPGGA, GPRMC, etc.)
 * transmitted over a hardware UART port.
 */
class GPS : public ISensor
{
public:
    /**
     * @brief Construct a new GPS sensor object.
     * @param tx_pin ESP32 pin connected to the GPS module's RX pin.
     * @param rx_pin ESP32 pin connected to the GPS module's TX pin.
     */
    GPS(int tx_pin, int rx_pin);

    ~GPS();

    /**
     * @brief Initializes the UART driver and starts the background NMEA parser.
     * Also updates the internal state via ISensor::setInitialized().
     * * @return true on successful initialization, false otherwise.
     */
    bool init() override;

    /**
     * @brief Checks the validity of the current GPS fix.
     * In ESP-IDF, data is parsed asynchronously in the background. 
     * This method simply checks if we have a valid 2D or 3D fix.
     * * @return true if a valid fix exists, false otherwise.
     */
    bool updateData() override;

    /**
    * @brief Getter for the sensor data (Thread-safe)
    * * @return a copy of the latest GPSData structure
     */
    GPSData getData();

private:
    // Static event handler required by the ESP-IDF Event Loop
    static void gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

    int _tx_pin;
    int _rx_pin;
    
    nmea_parser_handle_t _nmea_hdl;
    GPSData _data;
    
    // Mutex to protect _data from concurrent read/write operations
    // between the background parser task and the main application loop.
    SemaphoreHandle_t _data_mutex; 
};