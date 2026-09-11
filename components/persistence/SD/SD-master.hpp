#pragma once

#include <string>
#include <stdio.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "SerialLogger.hpp"
#include "pins.h"
#include <SPIBus.hpp>
#include "IStorage.hpp"
#include "SerialLogger.hpp"

/**
 * @brief Class to handle SD card operations.
 * */
class SD : public IStorage
{
private:
    sdmmc_card_t *_card = nullptr;
    FILE *_file = nullptr;
    bool _fileInitialized = false;
    const std::string _mount_point = "/sdcard";

public:
    bool init(SPIBus* bus, gpio_num_t cs_pin);

    /**
     * @brief Get the full path for a given filename on the SD card.
     * @param filename The name of the file (can be relative or absolute).
     * @return The full path to the file on the SD card.
     */
    std::string getFullPath(const std::string& filename) const;


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
    bool writeFile(const char* filename, const uint8_t* data, size_t length) override;
    
    /**
     * @brief Append a string of data to a file.
     */
    bool appendFile(const char* filename, const uint8_t* data, size_t length) override;
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
    size_t listFiles(StorageFileInfo* files, size_t capacity) override;
    bool readFileChunk(const char* filename, size_t offset, uint8_t* buffer,
                       size_t capacity, size_t& bytesRead, size_t& fileSize) override;
    bool deleteFile(const char* filename) override;

    bool isInitialized() const override { return _fileInitialized; }

    /**
     * @brief Get a pointer to the currently open file.
     */
    FILE* getFile() { return _file; }
};
