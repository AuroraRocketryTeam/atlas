#pragma once

#include <cstring>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "BaseTask.hpp"
#include "RocketModel.hpp"
#include "RocketLogger.hpp"
#include <config.h>
#include <SerialLogger.hpp>

class IStateMachine;

// Persistent recorder state composed into RuntimeConfig.
struct StorageLoggingConfig {
    char flight_filename[40];
    char last_flight_filename[40];
};

/**
 * @brief Class to implement a generic storage logging task using JSONL architecture.
 */
class StorageLoggingTask : public BaseTask {
public:
    /**
     * @brief Construct a new StorageLoggingTask object
     *
     * @param rocketModel The shared pointer to the rocket model storage wrapper
     * @param logger The shared pointer to the RocketLogger instance
     */
    StorageLoggingTask(std::shared_ptr<RocketModel> rocketModel,
                       std::shared_ptr<RocketLogger> logger,
                       IStateMachine* fsm);

    ~StorageLoggingTask() override;

    /** Prepare and persist the next free filename before RuntimeConfig is locked. */
    bool prepareTelemetryFilename();

    /** Called by the FSM after locking config, before entering READY. */
    bool prepareFlightLog();
    /** Activate a prepared log only after the FSM can commit READY. */
    bool activateFlightLog();
    void markFlightComplete() {
        if (logger) logger->stopSensorLogging();
    }
    static bool tryBeginFileOperation();
    static void endFileOperation();
    static bool isRecordingActive();

protected:
    void taskFunction() override;
    void onTaskStop() override;

private:
    std::shared_ptr<RocketModel> rocketModel;
    std::shared_ptr<RocketLogger> logger;
    IStateMachine* fsm;
    bool storageInitialized = false;
    bool flightLogPrepared = false;
    static std::atomic_bool fileOperationBusy;
    static std::atomic_bool recordingActive;

    static constexpr size_t TELEMETRY_FILENAME_SIZE = 40;
    char telemetryFilename[TELEMETRY_FILENAME_SIZE] = {};
    
    // Fixed-size memory block allocated once when the task is created
    static constexpr size_t WRITE_BUFFER_SIZE = 4096; 
    uint8_t writeBuffer[WRITE_BUFFER_SIZE];
    
    // Tracks how many bytes in the buffer are currently waiting to be written
    size_t pendingBytesToWrite = 0;

    static constexpr uint32_t FLUSH_TIMEOUT_MS = 1000;
    static constexpr int BATCH_ENTRY_THRESHOLD = 10;
    
};
