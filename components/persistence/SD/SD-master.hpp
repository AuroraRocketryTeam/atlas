#pragma once

#include <string>
#include <stdio.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "Logger.hpp"
#include "pins.h"
#include <SPIBus.hpp>
#include "IStorage.hpp"

/**
 * @brief Class to handle SD card operations.
 * */
class SD : public IStorage
{
private:
    sdmmc_card_t *card = nullptr;
    FILE *file = nullptr;
    bool fileInitialized = false;
    const std::string mount_point = "/sdcard";

public:
    bool init(SPIBus* bus, gpio_num_t cs_pin);

    /**
     * @brief Initialize the SD card.
     */
    bool openFile(const char* filename) override;

    /**
     * @brief Close the currently opened file.
     */
    bool closeFile() override;

    /**
     * @brief Write a string of data to a file.
     */
    bool writeFile(const char* filename, const char* content) override;

    /**
     * @brief Append a string of data to a file.
     */
    bool appendFile(const char* filename, const char* content) override;

    /**
     * @brief Read the whole content of a file.
     * @return A std::string with the contents of the file, or empty string if there was an error.
     */
    std::string readFile(const char* filename) override;
    
    /**
     * @brief Clear all contents of the SD card.
     */
    bool clearMemory() override;

    /**
     * @brief Check if a file exists on the SD card.
     */
    bool fileExists(const char* filename) override;
    
    /**
     * @brief Read a single line from the currently open file.
     * @return A std::string containing the next line, or an empty string if EOF or error.
     */
    std::string readLine() override;

    bool isInitialized() const override { return fileInitialized; }

    /**
     * @brief Get a pointer to the currently open file.
     */
    FILE* getFile() { return file; }
};