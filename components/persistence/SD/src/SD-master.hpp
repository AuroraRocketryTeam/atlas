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
 * 
 */
class SD : public IStorage
{
private:
    sdmmc_card_t *card = nullptr;
    FILE *file = nullptr;
    bool fileInitialized = false;
    const std::string mount_point = "/sdcard";

public:
    bool init(SPIBus* bus);

    /**
     * @brief Initialize the SD card.
     * 
     * @param filename The name of the file to open.
     * @return true if the file get opened successfully
     * @return false if the file could not be opened
     */
    bool openFile(const char* filename) override;

    /**
     * @brief Close the currently opened file.
     * 
     * @return true if the file was closed successfully
     * @return false if the file could not be closed
     */
    bool closeFile() override;

    /**
     * @brief Write a string of data to a file.
     * 
     * @param filename The name of the file to write to.
     * @param content The content to write to the file.
     * @return true if the file was written successfully
     * @return false if the file could not be written
     */
    bool writeFile(const char* filename, const char* content) override;  // se true file trovato e scritto, se false file non trovato

    /**
     * @brief Append a string of data to a file.
     * 
     * @param filename The name of the file to append to.
     * @param content The content to append to the file.
     * @return true if the content was appended successfully
     * @return false if there was an error
     */
    bool appendFile(const char* filename, const char* content) override; // se true contenuto aggiunto al file, se false errore

    /**
     * @brief Read the whole content of a file.
     * 
     * @param filename The name of the file to read.
     * @return A pointer to the contents of the file, or nullptr if there was an error.
     */
    char *readFile(const char* filename) override;                                                     // stampa il contenuto del file. ritorna true se tutto ok senno no
    
    /**
     * @brief Clear all contents of the SD card.
     * 
     * @return true if the SD card was cleared successfully
     * @return false if there was an error
     */
    bool clearMemory() override;                                                                           // cancella tutto il contenuto dell'sd

    /**
     * @brief Check if a file exists on the SD card.
     * 
     * @param filename The name of the file to check.
     * @return true if the file exists
     * @return false if the file does not exist
     */
    bool fileExists(const char* filename) override;                                                    // ritorna true se il file esiste, false se non esiste
    
    /**
     * @brief Read a single line from the currently open file.
     * 
     * @return A std::string containing the next line, or an empty string if EOF or error.
     */
    char* readLine() override;

    bool isInitialized() const override { return fileInitialized; }

    /**
     * @brief Get a pointer to the currently open file.
     * 
     * @return A pointer to the currently open file, or nullptr if no file is open.
     */
    FILE* getFile() { return file; }                                                        // ritorna il puntatore al file aperto
};