#include "SD-master.hpp"
#include <sys/stat.h>
#include <dirent.h>

/**
 * @brief A wrapper for SD card initialization
 *
 * @return true if the SD card is initialized, false otherwise
 */
bool SD::init()
{
    esp_err_t ret;
    
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {};
    mount_config.format_if_mount_failed = false; // if mount fails, do not format the card
    mount_config.max_files = 5; // max number of open files simultaneously
    mount_config.allocation_unit_size = 16 * 1024;

    LOG_INFO("SD-Task", "Initializing SD card");

    // Use settings defined above to initialize SD card and mount FAT filesystem
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = SD_SI;
    bus_cfg.miso_io_num = SD_SO;
    bus_cfg.sclk_io_num = SD_CLK;
    bus_cfg.quadwp_io_num = SD_QUADWP;
    bus_cfg.quadhd_io_num = SD_QUADHD;
    bus_cfg.max_transfer_sz = SD_MAX_TRANSFER_SIZE;

    ret = spi_bus_initialize(static_cast<spi_host_device_t>(host.slot), &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        LOG_ERROR("SD-Task", "Failed to initialize bus.");
        return false;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = (gpio_num_t)SD_CS;
    slot_config.host_id = static_cast<spi_host_device_t>(host.slot);

    LOG_INFO("SD-Task", "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point.c_str(), &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            LOG_ERROR("SD-Task", "Failed to mount filesystem.");
        } else {
            LOG_ERROR("SD-Task", "Failed to initialize the card (ESP_ERR: %d).", ret);
        }
        return false;
    }

    this->fileInitialized = true;
    return true;
}

/**
 * @brief File open wrapper
 *
 * @param filename
 * @return true if the file is opened, false otherwise
 */
bool SD::openFile(std::string filename)
{
    std::string full_path = mount_point + "/" + filename;
    
    if (this->file != nullptr) {
        fclose(this->file);
        this->file = nullptr;
    }

    this->file = fopen(full_path.c_str(), "a+");
    if (this->file == nullptr) {
        LOG_ERROR("SD-Task", "Failed to open file for appending/reading");
        return false;
    }

    fseek(this->file, 0, SEEK_SET);

    return true;
}

/**
 * @brief File close wrapper
 *
 * @return true if the file is closed, false otherwise.
 */
bool SD::closeFile()
{
    if (this->file == nullptr)
    {
        return false;
    }
    fclose(this->file);
    this->file = nullptr;
    return true;
}

/**
 * @brief Writes content to a file
 *
 * @param filename the file to write to
 * @param content  the content to write
 * @return true if the file is written, false otherwise
 */
bool SD::writeFile(std::string filename, std::variant<std::string, const char *> content)
{
    std::string full_path = mount_point + "/" + filename;

    // overwrite mode
    FILE* temp_file = fopen(full_path.c_str(), "w");
    if (temp_file == nullptr)
    {
        return false;
    }
    
    const char *data;
    if (std::holds_alternative<std::string>(content))
    {
        data = std::get<std::string>(content).c_str();
    }
    else
    {
        data = std::get<const char *>(content);
    }
    
    fputs(data, temp_file);
    fclose(temp_file);
    return true;
}

/**
 * @brief Appends content to a file
 *
 * @param filename the file to append to
 * @param content  the content to append
 * @return true if the content is appended, false otherwise
 */
bool SD::appendFile(std::string filename, std::variant<std::string, const char *> content)
{
    if (this->file == nullptr)
    {
        if(!this->openFile(filename))
        {
            return false;
        }
    }
    
    fseek(this->file, 0, SEEK_END);
    
    const char *data;
    if (std::holds_alternative<std::string>(content))
    {
        data = std::get<std::string>(content).c_str();
    }
    else
    {
        data = std::get<const char *>(content);
    }
    
    fputs(data, this->file);
    fflush(this->file);
    return true;
}

/**
 * @brief Reads the content of a file from start to end.
 * @note You need to free the memory after using the content.
 *
 * @return char* the content of the file.
 */
char *SD::readFile(std::string filename)
{
    if (this->file == nullptr)
    {
        if(!this->openFile(filename))
        {
            return nullptr;
        }
    }

    fseek(this->file, 0, SEEK_END);
    long fileSize = ftell(this->file);
    fseek(this->file, 0, SEEK_SET);

    if (fileSize <= 0) {
        return nullptr;
    }

    char *content = (char *)malloc(fileSize + 1);
    if (content == nullptr)
    {
        return nullptr;
    }

    size_t result = fread(content, 1, fileSize, this->file);
    content[result] = '\0';

    return content;
}

/**
 * @brief Deletes all files from the SD card.
 *
 * @return true if the SD card is cleared, false otherwise.
 */
bool SD::clearSD()
{
    DIR *dir = opendir(mount_point.c_str());
    if (!dir) {
        return false;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_type == DT_REG) {
            std::string file_path = mount_point + "/" + entry->d_name;
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
bool SD::fileExists(std::string filename)
{
    std::string full_path = mount_point + "/" + filename;
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
    if (this->file == nullptr) {
        LOG_INFO("SD-Task", "File not open");
        return "";
    }

    std::string str = "";    
    char ch;
    
    while (fread(&ch, 1, 1, this->file) == 1) {
        if (ch == '|') { // Based on original logic
            break;
        }
        if (ch != '\r') {
            str += ch;
        }
    }
    
    return str;
}

