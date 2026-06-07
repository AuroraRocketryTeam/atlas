#pragma once

#include <string>

/**
 * @brief Abstract interface for file system storage.
 */
class IStorage {
public:
    virtual ~IStorage() = default;

    virtual bool openFile(const char* filename) = 0;
    virtual bool closeFile() = 0;
    virtual bool writeFile(const char* filename, const uint8_t* data, size_t length) = 0;
    virtual bool appendFile(const char* filename, const uint8_t* data, size_t length) = 0;
    
    virtual std::string readFile(const char* filename) = 0;
    virtual std::string readLine() = 0;

    virtual bool clearMemory() = 0;
    virtual bool fileExists(const char* filename) = 0;

    virtual bool isInitialized() const = 0;
};