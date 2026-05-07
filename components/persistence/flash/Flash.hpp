#pragma once

#include <string>
#include <fstream>
#include <stdio.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_flash.h"
#include "esp_partition.h"
#include "SPIBus.hpp"

class Flash
{
private:

    SPIBus* _spi_bus = nullptr;
    gpio_num_t _cs_pin = GPIO_NUM_NC;
    gpio_num_t _hold_pin = GPIO_NUM_NC;
    gpio_num_t _wp_pin = GPIO_NUM_NC;

    // VFS mount point for LittleFS
    const std::string _mount_point = "/ext";
    const std::string _partition_label = "flash_memory";

    // ESP-IDF hardware and partition handles
    esp_flash_t* _ext_flash = nullptr;
    const esp_partition_t* _ext_part = nullptr;
    bool _initialized = false;

    // State used by openFile()/readLine() compatibility path
    std::string _open_filename;
    std::ifstream _active_stream;

    // Helper to format absolute paths for the VFS
    std::string getFullPath(const std::string& filename) const;

    bool isConfigured() const;
    
    // Drive HOLD/WP pins high to keep chip active
    bool configureHoldAndWpPins() const;

public:
    Flash() = default;
    ~Flash();

    /**
     * @brief Initializes the SPI bus, binds the flash chip, and mounts LittleFS.
     * @return true on success.
     */
    bool init();

    /**
     * @brief Configure SPI bus and flash-related pins, then initialize flash.
     *
     * @param bus      Already-initialized SPI bus used by external devices.
     * @param cs_pin   Chip select pin for external flash.
     * @param hold_pin HOLD pin for external flash.
     * @param wp_pin   WP pin for external flash.
     */
    bool init(SPIBus* bus, gpio_num_t cs_pin, gpio_num_t hold_pin, gpio_num_t wp_pin);

    /**
     * @brief Opens a file stream for sequential reading via readLine().
     */
    bool openFile(std::string filename);

    /**
     * @brief Closes the currently opened file stream.
     */
    bool closeFile();

    /**
     * @brief Write a string of data to a file (overwrites existing).
     */
    bool writeFile(std::string filename, std::string content);

    /**
     * @brief Write a string of data to a file (overwrites existing).
     */
    bool writeFile(std::string filename, const char* content);

    /**
     * @brief Append a string of data to a file.
     */
    bool appendFile(std::string filename, std::string content);
    
    /**
     * @brief Append a string of data to a file.
     */
    bool appendFile(std::string filename, const char * content);

    /**
     * @brief Read the whole content of a file.
     * @note Caller is responsible for freeing the returned memory using `delete[]`.
     */
    char *readFile(std::string filename);
    
    /**
     * @brief Clear external flash by formatting the LittleFS partition.
     */
    bool clearFlash();

    /**
     * @brief Check if a file exists on the flash.
     */
    bool fileExists(std::string filename);
    
    /**
     * @brief Read a single line from the currently open file.
     * * Streaming directly off the flash eliminates the RAM-spike vulnerability.
     */
    std::string readLine();
};