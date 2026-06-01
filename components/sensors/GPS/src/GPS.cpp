#include "GPS.hpp"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "GPS_SENSOR";

GPS::GPS(int tx_pin, int rx_pin) : _tx_pin(tx_pin), _rx_pin(rx_pin), _nmea_hdl(nullptr)
{
    // Initialize the data object and the FreeRTOS mutex
    _data_mutex = xSemaphoreCreateMutex();
}

GPS::~GPS()
{
    if (_nmea_hdl) {
        nmea_parser_remove_handler(_nmea_hdl, gps_event_handler);
        nmea_parser_deinit(_nmea_hdl);
    }
    if (_data_mutex) {
        vSemaphoreDelete(_data_mutex);
    }
}

bool GPS::init()
{
    // Configure standard UART parameters for NMEA (9600 baud is standard for u-blox defaults)
    nmea_parser_config_t config = {
        .uart = {
            .uart_port = UART_NUM_1,   // Use UART1 (UART0 is usually for logging/flashing)
            .rx_pin = static_cast<uint32_t>(_rx_pin),
            .baud_rate = 9600,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .event_queue_size = 16
        }
    };

    // Initialize the ESP-IDF background parser
    _nmea_hdl = nmea_parser_init(&config);
    if (!_nmea_hdl) {
        ESP_LOGE(TAG, "Failed to initialize NMEA parser");
        setInitialized(false);
        return false;
    }

    // Register our static event handler to listen for parsed GPS data
    esp_err_t err = nmea_parser_add_handler(_nmea_hdl, gps_event_handler, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add NMEA handler");
        setInitialized(false);
        return false;
    }

    ESP_LOGI(TAG, "GPS UART Parser initialized successfully on TX:%d / RX:%d", _tx_pin, _rx_pin);
    setInitialized(true);
    return true;
}

bool GPS::updateData()
{
    bool isValid = false;
    
    // Safely read the fix type using the mutex
    if (xSemaphoreTake(_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        isValid = (_data.fixType >= 2); 
        xSemaphoreGive(_data_mutex);
    }

    return isValid;
}

GPSData GPS::getData()
{
    // Create a safe copy so the background task doesn't overwrite
    // the data while the caller is actively trying to read it.
    GPSData safe_copy;
    
    if (xSemaphoreTake(_data_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        safe_copy = _data; // Copy the contents securely
        xSemaphoreGive(_data_mutex);
    } else {
        ESP_LOGW(TAG, "Failed to acquire GPS data mutex");
    }
    
    return safe_copy;
}

void GPS::gps_event_handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    GPS *gps_instance = static_cast<GPS*>(event_handler_arg);
    gps_t *gps = static_cast<gps_t*>(event_data);

    // Lock the mutex before writing new data to the shared _data object
    if (xSemaphoreTake(gps_instance->_data_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        
        gps_instance->_data.satellites = gps->sats_in_use;
        
        // Map ESP-IDF fix enum to integers
        if (gps->fix == GPS_FIX_INVALID) {
            gps_instance->_data.fixType = 0;
        } else {
            // Valid fix implied (minimum 3 for standard 3D NMEA parsing)
            gps_instance->_data.fixType = 3; 
        }

        if (gps->fix != GPS_FIX_INVALID) {
            gps_instance->_data.latitude = gps->latitude;
            gps_instance->_data.longitude = gps->longitude;
            gps_instance->_data.altitude = gps->altitude;
            // The parser provides speed in m/s. Convert to km/h.
            gps_instance->_data.ground_speed = gps->speed * 3.6; 
            gps_instance->_data.hdop = gps->dop_h;
            
            // esp_timer_get_time() returns microseconds since boot. Convert to ms.
            gps_instance->_data.timestamp = esp_timer_get_time() / 1000; 
        }
        
        xSemaphoreGive(gps_instance->_data_mutex);
    }
}