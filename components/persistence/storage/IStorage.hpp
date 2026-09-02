#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

struct StorageFileInfo {
    char name[64] = {};
    size_t size = 0;
};

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
    virtual size_t listFiles(StorageFileInfo* files, size_t capacity) = 0;
    virtual bool readFileChunk(const char* filename, size_t offset, uint8_t* buffer,
                               size_t capacity, size_t& bytesRead, size_t& fileSize) = 0;
    virtual bool deleteFile(const char* filename) = 0;

    virtual bool clearMemory() = 0;
    virtual bool fileExists(const char* filename) = 0;

    virtual bool isInitialized() const = 0;
};
