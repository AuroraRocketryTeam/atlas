#include "MirrorStorage.hpp"

#include <cstdlib>

void MirrorStorage::addStorage(const std::shared_ptr<IStorage>& storage) {
    if (storage) {
        _storages.push_back(storage);
    }
}

bool MirrorStorage::openFile(const char* filename) {
    if (filename == nullptr) {
        return false;
    }

    bool success = false;
    for (const auto& storage : _storages) {
        if (storage->isInitialized() && storage->openFile(filename)) {
            success = true;
        }
    }
    return success;
}

bool MirrorStorage::closeFile() {
    bool success = false;
    for (const auto& storage : _storages) {
        if (storage->isInitialized() && storage->closeFile()) {
            success = true;
        }
    }
    return success;
}

bool MirrorStorage::writeFile(const char* filename, const char* content) {
    if (filename == nullptr || content == nullptr) {
        return false;
    }

    bool success = false;
    for (const auto& storage : _storages) {
        if (storage->isInitialized() && storage->writeFile(filename, content)) {
            success = true;
        }
    }
    return success;
}

bool MirrorStorage::appendFile(const char* filename, const char* content) {
    if (filename == nullptr || content == nullptr) {
        return false;
    }

    bool success = false;
    for (const auto& storage : _storages) {
        if (storage->isInitialized() && storage->appendFile(filename, content)) {
            success = true;
        }
    }
    return success;
}

std::string MirrorStorage::readFile(const char* filename) {
    if (filename == nullptr) {
        return "";
    }

    for (const auto& storage : _storages) {
        if (storage->isInitialized() && storage->fileExists(filename)) {
            std::string data = storage->readFile(filename);
            if (!data.empty()) {
                return data;
            }
        }
    }
    return "";
}

std::string MirrorStorage::readLine() {
    for (const auto& storage : _storages) {
        if (storage->isInitialized()) {
            std::string line = storage->readLine();
            if (!line.empty()) {
                return line;
            }
        }
    }
    return "";
}

bool MirrorStorage::clearMemory() {
    bool success = false;
    for (const auto& storage : _storages) {
        if (storage->isInitialized() && storage->clearMemory()) {
            success = true;
        }
    }
    return success;
}

bool MirrorStorage::fileExists(const char* filename) {
    if (filename == nullptr) {
        return false;
    }

    for (const auto& storage : _storages) {
        if (storage->isInitialized() && storage->fileExists(filename)) {
            return true;
        }
    }
    return false;
}

bool MirrorStorage::isInitialized() const {
    for (const auto& storage : _storages) {
        if (storage->isInitialized()) {
            return true;
        }
    }
    return false;
}
