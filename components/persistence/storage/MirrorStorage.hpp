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
    bool writeFile(const char* filename, const char* content) override;
    bool appendFile(const char* filename, const char* content) override;

    std::string readFile(const char* filename) override;
    std::string readLine() override;

    bool clearMemory() override;
    bool fileExists(const char* filename) override;
    bool isInitialized() const override;
};