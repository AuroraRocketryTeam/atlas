#include "Flash.hpp"
#include <algorithm>
#include <cstdlib>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <cstring>
#include <utility>

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "esp_flash_spi_init.h"
#include "esp_partition.h"
#include "esp_littlefs.h"
#include "esp_check.h"
#include "esp_log.h"
#include "SPIBus.hpp"

static const char* TAG = "Flash";

Flash::~Flash() {
    closeFile();

    // Unmount filesystem if it was successfully registered
    if (_initialized) {
        esp_vfs_littlefs_unregister(_partition_label.c_str());
        _initialized = false;
    }
    
    // Unregister external partition if the pointer is valid
    if (_ext_part != nullptr) {
        esp_partition_deregister_external(_ext_part);
        _ext_part = nullptr;
    }
    
    // Remove device from SPI bus if the pointer is valid
    if (_ext_flash != nullptr) {
        spi_bus_remove_flash_device(_ext_flash);
        _ext_flash = nullptr;
    }
}

std::string Flash::getFullPath(const std::string& filename) const {
    // Ensure filename starts with a slash
    if (filename.empty() || filename[0] != '/') {
        return _mount_point + "/" + filename;
    }
    return _mount_point + filename;
}

bool Flash::isConfigured() const {
    return _spi_bus != nullptr && _cs_pin != GPIO_NUM_NC && _hold_pin != GPIO_NUM_NC && _wp_pin != GPIO_NUM_NC;
}

bool Flash::configureHoldAndWpPins() const {
    uint64_t pin_mask = 0;
    if (_hold_pin != GPIO_NUM_NC) pin_mask |= (1ULL << _hold_pin);
    if (_wp_pin != GPIO_NUM_NC) pin_mask |= (1ULL << _wp_pin);

    // If neither pin is used, just return true
    if (pin_mask == 0) return true;

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = pin_mask;
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;

    if (gpio_config(&cfg) != ESP_OK) return false;
    
    if (_hold_pin != GPIO_NUM_NC) gpio_set_level(_hold_pin, 1);
    if (_wp_pin != GPIO_NUM_NC) gpio_set_level(_wp_pin, 1);
    return true;
}

bool Flash::init(SPIBus* bus, gpio_num_t cs_pin, gpio_num_t hold_pin, gpio_num_t wp_pin) {
    _spi_bus = bus;
    _cs_pin = cs_pin;
    _hold_pin = hold_pin;
    _wp_pin = wp_pin;
    return init();
}

bool Flash::init() {
    if (_initialized) return true;

    if (!isConfigured()) {
        ESP_LOGE(TAG, "Flash init called without SPI/pin configuration");
        return false;
    }

    if (!_spi_bus->is_initialized()) {
        ESP_LOGE(TAG, "SPI bus is not initialized");
        return false;
    }

    if (!configureHoldAndWpPins()) {
        ESP_LOGE(TAG, "Failed to configure HOLD/WP GPIOs");
        return false;
    }

    // Add the flash device to the SPI bus.
    esp_flash_spi_device_config_t dev_cfg = {};
    dev_cfg.host_id = _spi_bus->get_host();
    dev_cfg.cs_id = 0;
    dev_cfg.cs_io_num = _cs_pin;
    dev_cfg.io_mode = SPI_FLASH_FASTRD; 
    dev_cfg.freq_mhz = ESP_FLASH_40MHZ;

    esp_err_t err = spi_bus_add_flash_device(&_ext_flash, &dev_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Add flash device failed: %s", esp_err_to_name(err));
        _ext_flash = nullptr;
        return false;
    }

    err = esp_flash_init(_ext_flash);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Flash chip init failed: %s", esp_err_to_name(err));
        spi_bus_remove_flash_device(_ext_flash);
        _ext_flash = nullptr;
        return false;
    }

    uint32_t capacity = 0;
    err = esp_flash_get_size(_ext_flash, &capacity);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read flash size: %s", esp_err_to_name(err));
        spi_bus_remove_flash_device(_ext_flash);
        _ext_flash = nullptr;
        return false;
    }
    
    ESP_LOGI(TAG, "External flash initialized. Capacity: %lu bytes", (unsigned long)capacity);

    // Register a virtual partition for the whole chip
    err = esp_partition_register_external(_ext_flash, 0, capacity, _partition_label.c_str(), 
                                          ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, &_ext_part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register external partition: %s", esp_err_to_name(err));
        spi_bus_remove_flash_device(_ext_flash);
        _ext_flash = nullptr;
        return false;
    }

    // Mount LittleFS on the external partition
    esp_vfs_littlefs_conf_t lfs_conf = {};
    lfs_conf.base_path = _mount_point.c_str();
    lfs_conf.partition_label = _partition_label.c_str();
    lfs_conf.format_if_mount_failed = true;
    lfs_conf.dont_mount = false;

    err = esp_vfs_littlefs_register(&lfs_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount LittleFS: %s", esp_err_to_name(err));
        esp_partition_deregister_external(_ext_part);
        _ext_part = nullptr;
        spi_bus_remove_flash_device(_ext_flash);
        _ext_flash = nullptr;
        return false;
    }

    _initialized = true;
    return true;
}

bool Flash::openFile(const char* filename) {
    if (filename == nullptr) return false;
    if (!_initialized && !init()) return false;

    closeFile();
    _open_filename = getFullPath(filename);
    
    // Open for reading ("r"). 
    _active_file = fopen(_open_filename.c_str(), "r");

    // If it doesn't exist, safely create it in append/update mode ("a+") 
    // to guarantee we never truncate existing data.
    if (!_active_file) {
        _active_file = fopen(_open_filename.c_str(), "a+");
        // Reset read pointer to the beginning such that the append will work fine
        if (_active_file) fseek(_active_file, 0, SEEK_SET); 
    }

    return _active_file != nullptr;
}

bool Flash::closeFile() {
    if (_active_file) {
        fclose(_active_file);
        _active_file = nullptr;
    }
    _open_filename.clear();
    return true;
}

bool Flash::writeFile(const char* filename, const uint8_t* data, size_t length) {
    // Check parameters
    if (filename == nullptr || data == nullptr || length == 0) {
        LOG_ERROR("Flash", "Invalid arguments (null pointers or zero length).");
        return false;
    }

    if (!_initialized && !init()) {
        LOG_ERROR("Flash", "Failed to initialize Flash subsystem.");
        return false;
    }

    std::string fullPath = getFullPath(filename);
    FILE* f = fopen(fullPath.c_str(), "w");
    
    // Check if file opened successfully
    if (!f) {
        LOG_ERROR("Flash", "Could not open '%s' - %s", fullPath.c_str(), strerror(errno));
        return false;
    }

    // Write and check the result
    size_t written = fwrite(data, 1, length, f);
    if (written != length) {
        LOG_ERROR("Flash", "Write incomplete on '%s'. Tried to write %zu bytes, but only wrote %zu. Reason: %s", 
                  fullPath.c_str(), length, written, strerror(errno));
        
        fclose(f);
        return false;
    }

    // Check fclose
    if (fclose(f) != 0) {
        LOG_ERROR("Flash", "Failed to close/flush file '%s' - %s", fullPath.c_str(), strerror(errno));
        return false;
    }

    return true;
}

bool Flash::appendFile(const char* filename, const uint8_t* data, size_t length) {
    // Validate inputs
    if (filename == nullptr || data == nullptr || length == 0) {
        LOG_ERROR("Flash", "Invalid arguments (null pointers or zero length).");
        return false;
    }

    if (!_initialized && !init()) {
        LOG_ERROR("Flash", "Failed to initialize Flash subsystem.");
        return false;
    }

    std::string fullPath = getFullPath(filename);
    FILE* f = fopen(fullPath.c_str(), "a");
    
    // Check if file opened successfully
    if (!f) {
        LOG_ERROR("Flash", "Could not open '%s' for appending - %s", fullPath.c_str(), strerror(errno));
        return false;
    }

    // Perform the write and check the result
    size_t written = fwrite(data, 1, length, f);
    if (written != length) {
        LOG_ERROR("Flash", "Append incomplete on '%s'. Tried to append %zu bytes, but only wrote %zu. Reason: %s", 
                  fullPath.c_str(), length, written, strerror(errno));
        
        fclose(f);
        return false;
    }

    // Check fclose as well (crucial for ensuring appended data is flushed)
    if (fclose(f) != 0) {
        LOG_ERROR("Flash", "Failed to close/flush file '%s' after appending - %s", fullPath.c_str(), strerror(errno));
        return false;
    }

    return true;
}

std::string Flash::readFile(const char* filename) {
    if (filename == nullptr) return "";
    if (!_initialized && !init()) return "";

    FILE* f = fopen(getFullPath(filename).c_str(), "rb");
    if (!f) return "";

    
    
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);

    if (size <= 0) {
        fclose(f);
        return "";
    }

    std::string buffer;
    buffer.resize(size);
    
    size_t read_bytes = fread(&buffer[0], 1, size, f);
    fclose(f);
    
    if (read_bytes == size) {
        return buffer;
    }
    return "";
}

std::string Flash::readLine() {
    if (!_active_file) return "";

    std::string line;
    // Pre-allocate a safe baseline to prevent early heap fragmentation
    line.reserve(1024);
    char buffer[256]; 

    while (fgets(buffer, sizeof(buffer), _active_file) != nullptr) {
        line += buffer;
        
        if (!line.empty() && line.back() == '\n') {
            break; 
        }
    }

    return line;
}

bool Flash::clearMemory() {
    if (!_initialized && !init()) return false;

    ESP_LOGI(TAG, "Formatting LittleFS partition...");
    esp_err_t err = esp_littlefs_format(_partition_label.c_str());
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Format failed: %s", esp_err_to_name(err));
        return false;
    }
    
    ESP_LOGI(TAG, "Format successful.");
    return true;
}

bool Flash::fileExists(const char* filename) {
    if (filename == nullptr) return false;
    if (!_initialized && !init()) return false;

    struct stat st;
    return stat(getFullPath(filename).c_str(), &st) == 0;
}

size_t Flash::listFiles(StorageFileInfo* files, size_t capacity) {
    if (files == nullptr || capacity == 0 || (!_initialized && !init())) return 0;
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

bool Flash::readFileChunk(const char* filename, size_t offset, uint8_t* buffer,
                          size_t capacity, size_t& bytesRead, size_t& fileSize) {
    bytesRead = 0;
    fileSize = 0;
    if (filename == nullptr || buffer == nullptr || capacity == 0 || (!_initialized && !init())) return false;
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

bool Flash::deleteFile(const char* filename) {
    return filename != nullptr && _initialized && unlink(getFullPath(filename).c_str()) == 0;
}
