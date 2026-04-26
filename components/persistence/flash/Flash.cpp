#include "Flash.hpp"
#include <cstdlib>
#include <sys/stat.h>
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
    _active_stream.open(_open_filename);

    // If the file doesn't exist, it is created
    if (!_active_stream.is_open()) {
        std::ofstream create(_open_filename);
        create.close();
        _active_stream.open(_open_filename);
    }

    return _active_stream.is_open();
}

bool Flash::closeFile() {
    if (_active_stream.is_open()) {
        _active_stream.close();
    }
    _open_filename.clear();
    return true;
}

bool Flash::writeFile(const char* filename, const char* content) {
    if (filename == nullptr || content == nullptr) return false;
    if (!_initialized && !init()) return false;

    std::ofstream os(getFullPath(filename), std::ios::trunc | std::ios::out);
    if (!os.is_open()) return false;

    os << content;
    os.close();
    return os.good();
}

bool Flash::appendFile(const char* filename, const char* content) {
    if (filename == nullptr || content == nullptr) return false;
    if (!_initialized && !init()) return false;

    std::ofstream os(getFullPath(filename), std::ios::app | std::ios::out);
    if (!os.is_open()) return false;

    os << content;
    os.close();
    return os.good();
}

char* Flash::readFile(const char* filename) {
    if (filename == nullptr) return nullptr;
    if (!_initialized && !init()) return nullptr;

    std::ifstream is(getFullPath(filename), std::ios::in | std::ios::binary | std::ios::ate);
    if (!is.is_open()) return nullptr;

    std::streamsize size = is.tellg();
    is.seekg(0, std::ios::beg);

    if (size <= 0) {
        is.close();
        return nullptr;
    }

    char* buffer = new char[size + 1];
    if (is.read(buffer, size)) {
        buffer[size] = '\0';
        is.close();
        return buffer;
    }

    delete[] buffer;
    is.close();
    return nullptr;
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

char* Flash::readLine() {
    if (!_active_stream.is_open()) return nullptr;

    std::string line;
    if (std::getline(_active_stream, line)) {
        line += "\n";
        char* out = static_cast<char*>(malloc(line.size() + 1));
        if (out == nullptr) {
            return nullptr;
        }
        memcpy(out, line.c_str(), line.size() + 1);
        return out;
    }
    return nullptr;
}