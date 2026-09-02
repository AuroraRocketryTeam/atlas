#include "SD-master.hpp"
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>

/**
 * @brief A wrapper for SD card initialization
 *
 * @return true if the SD card is initialized, false otherwise
 */
bool SD::init(SPIBus* bus, gpio_num_t cs_pin)
{
    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false; // if mount fails, do not format the card
    mount_config.max_files = 5; // max number of open files simultaneously
    mount_config.allocation_unit_size = 16 * 1024;

    LOG_INFO("SD-Task", "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = bus->get_host();
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = cs_pin;
    slot_config.host_id = bus->get_host();

    LOG_INFO("SD-Task", "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(this->_mount_point.c_str(), &host, &slot_config, &mount_config, &_card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            LOG_ERROR("SD-Task", "Failed to mount filesystem.");
        } else {
            LOG_ERROR("SD-Task", "Failed to initialize the card (ESP_ERR: %d).", ret);
        }
        return false;
    }

    this->_fileInitialized = true;
    return true;
}

std::string SD::getFullPath(const std::string& filename) const {
    // Ensure filename starts with a slash
    if (filename.empty() || filename[0] != '/') {
        return this->_mount_point + "/" + filename;
    }
    return this->_mount_point + filename;
}

/**
 * @brief File open wrapper
 *
 * @param filename
 * @return true if the file is opened, false otherwise
 */
bool SD::openFile(const char* filename)
{
    if (filename == nullptr) {
        return false;
    }

    std::string full_path = getFullPath(filename);
    
    if (this->_file != nullptr) {
        fclose(this->_file);
        this->_file = nullptr;
    }

    this->_file = fopen(full_path.c_str(), "a+");
    if (this->_file == nullptr) {
        LOG_ERROR("SD-Task", "Failed to open file for appending/reading");
        return false;
    }

    fseek(this->_file, 0, SEEK_SET);

    return true;
}

/**
 * @brief File close wrapper
 *
 * @return true if the file is closed, false otherwise.
 */
bool SD::closeFile()
{
    if (this->_file == nullptr)
    {
        return false;
    }
    fclose(this->_file);
    this->_file = nullptr;
    return true;
}

bool SD::writeFile(const char* filename, const uint8_t* data, size_t length) {
    // Check parameters
    if (filename == nullptr || data == nullptr || length == 0) {
        LOG_ERROR("SD-Task", "Invalid arguments (null pointers or zero length).");
        return false;
    }

    if (!_card) {
        LOG_ERROR("SD-Task", "Failed to initialize SD card.");
        return false;
    }

    std::string fullPath = getFullPath(filename);

    FILE* f = fopen(fullPath.c_str(), "w");
    
    // Check if file opened successfully
    if (!f) {
        LOG_ERROR("SD-Task", "Could not open '%s' - %s", fullPath.c_str(), strerror(errno));
        return false;
    }

    // Write and check the result
    size_t written = fwrite(data, 1, length, f);
    if (written != length) {
        LOG_ERROR("SD-Task", "Write incomplete on '%s'. Tried to write %zu bytes, but only wrote %zu. Reason: %s", 
                  fullPath.c_str(), length, written, strerror(errno));
        
        fclose(f);
        return false;
    }

    // Check fclose
    if (fclose(f) != 0) {
        LOG_ERROR("SD-Task", "Failed to close/flush file '%s' - %s", fullPath.c_str(), strerror(errno));
        return false;
    }

    return true;
}

/**
 * @brief Appends content to a file
 *
 * @param filename the file to append to
 * @param content  the content to append
 * @return true if the content is appended, false otherwise
 */
bool SD::appendFile(const char* filename, const uint8_t* data, size_t length) {
    // Validate inputs
    if (filename == nullptr || data == nullptr || length == 0) {
        LOG_ERROR("SD-Task", "Invalid arguments (null pointers or zero length).");
        return false;
    }

    if (!_card) {
        LOG_ERROR("SD-Task", "Failed to initialize SD card.");
        return false;
    }

    std::string fullPath = getFullPath(filename);
    FILE* f = fopen(fullPath.c_str(), "a");
    
    // Check if file opened successfully
    if (!f) {
        LOG_ERROR("SD-Task", "Could not open '%s' for appending - %s", fullPath.c_str(), strerror(errno));
        return false;
    }

    // Perform the write and check the result
    size_t written = fwrite(data, 1, length, f);
    if (written != length) {
        LOG_ERROR("SD-Task", "Append incomplete on '%s'. Tried to append %zu bytes, but only wrote %zu. Reason: %s", 
                  fullPath.c_str(), length, written, strerror(errno));
        
        fclose(f);
        return false;
    }

    // Check fclose as well (crucial for ensuring appended data is flushed)
    if (fclose(f) != 0) {
        LOG_ERROR("SD-Task", "Failed to close/flush file '%s' after appending - %s", fullPath.c_str(), strerror(errno));
        return false;
    }

    return true;
}


/**
 * @brief Reads the content of a file from start to end.
 * @note You need to free the memory after using the content.
 *
 * @return char* the content of the file.
 */
std::string SD::readFile(const char* filename)
{
    if (filename == nullptr) {
        return "";
    }

    if (this->_file == nullptr)
    {
        if(!this->openFile(filename))
        {
            return "";
        }
    }

    fseek(this->_file, 0, SEEK_END);
    long fileSize = ftell(this->_file);
    fseek(this->_file, 0, SEEK_SET);

    if (fileSize <= 0) {
        return "";
    }

    std::string content;
    content.resize(fileSize);

    size_t result = fread(&content[0], 1, fileSize, this->_file);
    if (result != fileSize) {
        content.resize(result);
    }

    return content;
}

/**
 * @brief Deletes all files from the SD card.
 *
 * @return true if the SD card is cleared, false otherwise.
 */
bool SD::clearMemory()
{
    DIR *dir = opendir(this->_mount_point.c_str());
    if (!dir) {
        return false;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_REG) {
            std::string file_path = getFullPath(entry->d_name);
            unlink(file_path.c_str());
        }
    }
    closedir(dir);
    return true;
}

/**
 * @brief Checks if a file exists on the SD card.
 *
 * @param filename the file to check for existence
 * @return true if the file exists, false otherwise
 */
bool SD::fileExists(const char* filename)
{
    if (filename == nullptr) {
        return false;
    }

    std::string full_path = getFullPath(filename);
    struct stat st;
    if (stat(full_path.c_str(), &st) == 0) {
        return true;
    }
    return false;
}

/**
 * @brief Reads a single line from the currently open file
 *
 * @return String containing the next line, or empty String if EOF or error
 */
std::string SD::readLine() {
    if (this->_file == nullptr) {
        LOG_INFO("SD-Task", "File not open");
        return "";
    }

    std::string str = "";
    char ch;
    
    while (fread(&ch, 1, 1, this->_file) == 1) {
        if (ch == '|') {
            break;
        }
        if (ch != '\r') {
            str += ch;
        }
    }
    
    return str;
}

size_t SD::listFiles(StorageFileInfo* files, size_t capacity) {
    if (files == nullptr || capacity == 0 || !_fileInitialized) return 0;
    DIR* dir = opendir(_mount_point.c_str());
    if (dir == nullptr) return 0;

    size_t count = 0;
    while (count < capacity) {
        dirent* entry = readdir(dir);
        if (entry == nullptr) break;
        struct stat st = {};
        if (stat(getFullPath(entry->d_name).c_str(), &st) != 0 || !S_ISREG(st.st_mode)) continue;
        snprintf(files[count].name, sizeof(files[count].name), "%s", entry->d_name);
        files[count].size = static_cast<size_t>(st.st_size);
        ++count;
    }
    closedir(dir);
    return count;
}

bool SD::readFileChunk(const char* filename, size_t offset, uint8_t* buffer,
                       size_t capacity, size_t& bytesRead, size_t& fileSize) {
    bytesRead = 0;
    fileSize = 0;
    if (filename == nullptr || buffer == nullptr || capacity == 0 || !_fileInitialized) return false;
    FILE* file = fopen(getFullPath(filename).c_str(), "rb");
    if (file == nullptr) return false;
    if (fseek(file, 0, SEEK_END) != 0) { fclose(file); return false; }
    const long size = ftell(file);
    if (size < 0 || offset > static_cast<size_t>(size) || fseek(file, static_cast<long>(offset), SEEK_SET) != 0) {
        fclose(file);
        return false;
    }
    fileSize = static_cast<size_t>(size);
    bytesRead = fread(buffer, 1, std::min(capacity, fileSize - offset), file);
    const bool ok = !ferror(file);
    fclose(file);
    return ok;
}

bool SD::deleteFile(const char* filename) {
    return filename != nullptr && _fileInitialized && unlink(getFullPath(filename).c_str()) == 0;
}
