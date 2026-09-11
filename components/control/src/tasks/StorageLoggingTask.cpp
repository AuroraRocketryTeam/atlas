#include "StorageLoggingTask.hpp"
#include "RuntimeConfig.hpp"
#include "esp_task_wdt.h"
#include <cstdio>

StorageLoggingTask::StorageLoggingTask(std::shared_ptr<RocketModel> rocketModel,
                       std::shared_ptr<RocketLogger> logger)
    : BaseTask("StorageLoggingTask"),
      rocketModel(rocketModel),
      logger(logger)
{
    storageInitialized = this->rocketModel && this->rocketModel->isStorageInitialized();

    if (!storageInitialized) {
        LOG_ERROR("StorageLoggingTask", "Storage initialization failed or no storage provided!");
    } else {
        prepareTelemetryFilename();
        LOG_INFO("StorageLoggingTask", "Storage initialized successfully. Ready for JSONL logging.");
    }
}

StorageLoggingTask::~StorageLoggingTask() {
    stop();
}

bool StorageLoggingTask::prepareTelemetryFilename() {
    RuntimeConfig config;
    if (runtime_config_get(&config) != ESP_OK) {
        LOG_ERROR("StorageLoggingTask", "Cannot select flight filename: RuntimeConfig unavailable");
        return false;
    }

    unsigned long parsedFlightNumber = 1;
    (void)sscanf(config.storage_logging.flight_filename,
                 "flight_telemetry_%lu.jsonl", &parsedFlightNumber);
    uint32_t flightNumber = parsedFlightNumber <= UINT32_MAX
        ? static_cast<uint32_t>(parsedFlightNumber) : 1;
    bool configChanged = false;
    while (flightNumber != 0) {
        snprintf(telemetryFilename, sizeof(telemetryFilename),
                 "flight_telemetry_%06lu.jsonl", static_cast<unsigned long>(flightNumber));
        if (!rocketModel->storageFileExists(telemetryFilename)) {
            if (strcmp(config.storage_logging.flight_filename, telemetryFilename) != 0) {
                snprintf(config.storage_logging.flight_filename,
                         sizeof(config.storage_logging.flight_filename), "%s", telemetryFilename);
                configChanged = true;
            }
            if (configChanged &&
                runtime_config_record_flight_files(config.storage_logging.flight_filename,
                                                   config.storage_logging.last_flight_filename) != ESP_OK) {
                telemetryFilename[0] = '\0';
                LOG_ERROR("StorageLoggingTask", "Could not persist flight filename");
                return false;
            }
            LOG_INFO("StorageLoggingTask", "Recording flight to %s", telemetryFilename);
            return true;
        }
        snprintf(config.storage_logging.last_flight_filename,
                 sizeof(config.storage_logging.last_flight_filename), "%s", telemetryFilename);
        configChanged = true;
        ++flightNumber;
    }

    telemetryFilename[0] = '\0';
    LOG_ERROR("StorageLoggingTask", "No available flight telemetry filename");
    return false;
}

void StorageLoggingTask::taskFunction() {
    static constexpr uint32_t LOGGER_STATS_PERIOD_MS = 10000;
    TickType_t lastWriteTicks = xTaskGetTickCount();
    TickType_t lastStatsTicks = lastWriteTicks;
    uint32_t lastDroppedCount = logger ? logger->getDroppedCount() : 0;
    bool storageWasAvailable = storageInitialized;

    if (logger && storageWasAvailable) {
        logger->logInfo("StorageLoggingTask", "Flight recorder active");
    }
    if (telemetryFilename[0] == '\0') {
        LOG_ERROR("StorageLoggingTask", "Flight recorder has no filename prepared at boot");
    }

    while (running) {
        esp_task_wdt_reset();

        storageInitialized = rocketModel && rocketModel->isStorageInitialized();

        if (storageInitialized != storageWasAvailable) {
            if (storageInitialized) {
                if (logger) {
                    logger->logInfo("StorageLoggingTask", "Flight recorder storage restored");
                }
            } else {
                LOG_ERROR("StorageLoggingTask", "Flight recorder storage unavailable");
            }
            storageWasAvailable = storageInitialized;
        }

        // Handle Storage Disconnection
        if (!storageInitialized) {            
            // Drop pending old data
            pendingBytesToWrite = 0;
            lastDroppedCount = logger ? logger->getDroppedCount() : 0;
            lastStatsTicks = xTaskGetTickCount();
            
            vTaskDelay(pdMS_TO_TICKS(100)); 
            continue;
        }

        // Evaluate if we need to trigger a write cycle
        int currentLogCount = logger ? logger->getLogCount() : 0;
        TickType_t currentTicks = xTaskGetTickCount();
        bool timeoutReached = (currentTicks - lastWriteTicks) * portTICK_PERIOD_MS >= FLUSH_TIMEOUT_MS;

        const uint32_t statsElapsedMs = (currentTicks - lastStatsTicks) * portTICK_PERIOD_MS;
        if (logger && statsElapsedMs >= LOGGER_STATS_PERIOD_MS) {
            const uint32_t droppedTotal = logger->getDroppedCount();
            const uint32_t droppedDelta = droppedTotal - lastDroppedCount;
            if (droppedDelta > 0) {
                LOG_WARNING("StorageLoggingTask", "Logger lost %u records in %u ms while storage was active (total=%u)",
                            static_cast<unsigned>(droppedDelta),
                            static_cast<unsigned>(statsElapsedMs),
                            static_cast<unsigned>(droppedTotal));
                char message[96];
                snprintf(message, sizeof(message), "Lost %u records in %u ms (total=%u)",
                         static_cast<unsigned>(droppedDelta),
                         static_cast<unsigned>(statsElapsedMs),
                         static_cast<unsigned>(droppedTotal));
                logger->logWarning("StorageLoggingTask", message);
            }
            lastDroppedCount = droppedTotal;
            lastStatsTicks = currentTicks;
        }

        // We write if we have leftover data, OR if data is getting stale, OR if we have "enough" logs waiting
        bool shouldWrite = (pendingBytesToWrite > 0) || 
                        (timeoutReached && currentLogCount > 0) ||
                        (currentLogCount >= BATCH_ENTRY_THRESHOLD);

        // Execute Write
        if (shouldWrite && running && telemetryFilename[0] != '\0') {
            if (pendingBytesToWrite == 0 && logger) {
                // Just hand over the entire available buffer space!
                pendingBytesToWrite = logger->consumeBatch(writeBuffer, WRITE_BUFFER_SIZE);
            }

            if (pendingBytesToWrite > 0) {
                if (rocketModel->storageAppendFile(telemetryFilename, writeBuffer, pendingBytesToWrite)) {
                    pendingBytesToWrite = 0;
                    lastWriteTicks = xTaskGetTickCount();
                } else {
                    LOG_DEBUG("StorageLoggingTask", "Write failed. Retrying next loop.");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(80)); // 12.5 Hz
    }
}
