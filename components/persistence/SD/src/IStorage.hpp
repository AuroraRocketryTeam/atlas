#pragma once

/**
 * @brief Abstract interface for file system storage.
 */
class IStorage {
public:
    virtual ~IStorage() = default;

    virtual bool openFile(const char* filename) = 0;
    virtual bool closeFile() = 0;
    virtual bool writeFile(const char* filename, const char* content) = 0;
    virtual bool appendFile(const char* filename, const char* content) = 0;
    virtual char* readFile(const char* filename) = 0;

    virtual bool clearMemory() = 0;
    virtual bool fileExists(const char* filename) = 0;
    virtual char* readLine() = 0;

    virtual bool isInitialized() const = 0;
};
