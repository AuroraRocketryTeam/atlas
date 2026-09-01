#pragma once

#include "IStorage.hpp"
#include <memory>
#include <vector>
#include <string>

class MirrorStorage : public IStorage {
private:
    std::vector<std::shared_ptr<IStorage>> _storages;

public:
    MirrorStorage() = default;
    ~MirrorStorage() override = default;

    /**
     * @brief Add a storage device to the mirror pool.
     */
    void addStorage(const std::shared_ptr<IStorage>& storage);

    bool openFile(const char* filename) override;
    bool closeFile() override;
    bool writeFile(const char* filename, const uint8_t* data, size_t length) override;
    bool appendFile(const char* filename, const uint8_t* data, size_t length) override;
    std::string readFile(const char* filename) override;
    std::string readLine() override;
    size_t listFiles(StorageFileInfo* files, size_t capacity) override;
    bool readFileChunk(const char* filename, size_t offset, uint8_t* buffer,
                       size_t capacity, size_t& bytesRead, size_t& fileSize) override;
    bool deleteFile(const char* filename) override;

    bool clearMemory() override;
    bool fileExists(const char* filename) override;
    bool isInitialized() const override;
};
