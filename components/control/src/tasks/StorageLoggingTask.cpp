#include "StorageLoggingTask.hpp"
#include "RuntimeConfig.hpp"
#include "IStateMachine.hpp"
#include "esp_task_wdt.h"
#include <cstdio>

std::atomic_bool StorageLoggingTask::fileOperationBusy{false};
std::atomic_bool StorageLoggingTask::recordingActive{false};

StorageLoggingTask::StorageLoggingTask(std::shared_ptr<RocketModel> rocketModel,
                       std::shared_ptr<RocketLogger> logger,
                       IStateMachine* fsm)
    : BaseTask("StorageLoggingTask"),
      rocketModel(rocketModel),
      logger(logger),
      fsm(fsm)
{
    storageInitialized = this->rocketModel && this->rocketModel->isStorageInitialized();

    if (!storageInitialized) {
        LOG_ERROR("StorageLoggingTask", "Storage initialization failed or no storage provided!");
    } else {
        LOG_INFO("StorageLoggingTask", "Storage initialized successfully. Ready for JSONL logging.");
    }
}

StorageLoggingTask::~StorageLoggingTask() {
    stop();
}

bool StorageLoggingTask::tryBeginFileOperation() {
    bool expected = false;
    return fileOperationBusy.compare_exchange_strong(expected, true);
}

void StorageLoggingTask::endFileOperation() {
    fileOperationBusy.store(false);
}

bool StorageLoggingTask::isRecordingActive() {
    return recordingActive.load();
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
    while (flightNumber != 0) {
        snprintf(telemetryFilename, sizeof(telemetryFilename),
                 "flight_telemetry_%06lu.jsonl", static_cast<unsigned long>(flightNumber));
        bool exists = false;
        if (!rocketModel->storageFileExistsChecked(telemetryFilename, exists, 1000)) {
            LOG_ERROR("StorageLoggingTask", "Cannot inspect flight filename %s", telemetryFilename);
            telemetryFilename[0] = '\0';
            return false;
        }
        if (!exists) {
            if (strcmp(config.storage_logging.flight_filename, telemetryFilename) != 0) {
                snprintf(config.storage_logging.flight_filename,
                         sizeof(config.storage_logging.flight_filename), "%s", telemetryFilename);
                if (runtime_config_record_flight_files(config.storage_logging.flight_filename,
                                                       config.storage_logging.last_flight_filename) != ESP_OK) {
                    telemetryFilename[0] = '\0';
                    LOG_ERROR("StorageLoggingTask", "Could not persist flight filename");
                    return false;
                }
            }
            LOG_INFO("StorageLoggingTask", "Next flight filename: %s", telemetryFilename);
            return true;
        }
        ++flightNumber;
    }

    telemetryFilename[0] = '\0';
    LOG_ERROR("StorageLoggingTask", "No available flight telemetry filename");
    return false;
}

bool StorageLoggingTask::prepareFlightLog() {
    flightLogPrepared = false;
    bool exists = false;
    if (!logger || !rocketModel || !rocketModel->isStorageInitialized() ||
        telemetryFilename[0] == '\0' || recordingActive.load() ||
        !rocketModel->storageFileExistsChecked(telemetryFilename, exists, 1000) || exists) {
        LOG_ERROR("StorageLoggingTask", "Cannot prepare flight log: storage or filename unavailable");
        return false;
    }
    const RuntimeConfig config = runtime_config_get_flight_snapshot();
    if (!config.config_locked) {
        LOG_ERROR("StorageLoggingTask", "Cannot prepare flight log before configuration lock");
        return false;
    }
    // Fail before touching the event queue if the locked snapshot cannot fit.
    if (runtime_config_to_json(&config, reinterpret_cast<char*>(writeBuffer),
                               WRITE_BUFFER_SIZE - 64) != ESP_OK) {
        LOG_ERROR("StorageLoggingTask", "Locked RuntimeConfig is too large for a flight record");
        return false;
    }
    bool fileCreated = false;
    // Persist each event before removing its RAM copy. A failed preparation
    // leaves the config locked only until the FSM rejects READY_FOR_LAUNCH and unlocks it.
    while (logger->getEventCount() > 0) {
        const size_t length = logger->peekNextEvent(writeBuffer, WRITE_BUFFER_SIZE);
        if (length == 0) {
            LOG_ERROR("StorageLoggingTask", "Could not serialize preflight event");
            return false;
        }
        const bool written = fileCreated
            ? rocketModel->storageAppendFile(telemetryFilename, writeBuffer, length)
            : rocketModel->storageWriteFile(telemetryFilename, writeBuffer, length);
        if (!written) {
            LOG_ERROR("StorageLoggingTask", "Failed to write preflight events; partial file %s kept for diagnosis",
                      telemetryFilename);
            return false;
        }
        fileCreated = true;
        logger->discardNextEvent();
    }
    if (logger->getEventCount() > 0) {
        LOG_ERROR("StorageLoggingTask", "Preflight event queue could not be drained");
        return false;
    }

    int prefix = snprintf(reinterpret_cast<char*>(writeBuffer), WRITE_BUFFER_SIZE,
                          "{\"type\":\"CONFIG\",\"timestamp\":%lu,\"config\":",
                          static_cast<unsigned long>(Utils::millis()));
    if (prefix < 0 || static_cast<size_t>(prefix) + 3 >= WRITE_BUFFER_SIZE ||
        runtime_config_to_json(&config, reinterpret_cast<char*>(writeBuffer) + prefix,
                               WRITE_BUFFER_SIZE - prefix - 2) != ESP_OK) {
        LOG_ERROR("StorageLoggingTask", "Locked RuntimeConfig does not fit the flight record buffer");
        return false;
    }
    size_t length = static_cast<size_t>(prefix) +
                    strlen(reinterpret_cast<char*>(writeBuffer) + prefix);
    writeBuffer[length++] = '}';
    writeBuffer[length++] = '\n';
    const bool written = fileCreated
        ? rocketModel->storageAppendFile(telemetryFilename, writeBuffer, length)
        : rocketModel->storageWriteFile(telemetryFilename, writeBuffer, length);
    if (!written) {
        LOG_ERROR("StorageLoggingTask", "Failed to persist locked RuntimeConfig; partial file %s kept for diagnosis",
                  telemetryFilename);
        return false;
    }
    flightLogPrepared = true;
    LOG_INFO("StorageLoggingTask", "Flight log prepared: %s", telemetryFilename);
    return true;
}

bool StorageLoggingTask::activateFlightLog() {
    if (!flightLogPrepared || recordingActive.load()) {
        LOG_ERROR("StorageLoggingTask", "Cannot activate an unprepared flight log");
        return false;
    }
    // LATEST means the most recent flight admitted to READY, including one later aborted.
    // Keep the recorder inactive if this persistent marker cannot be written.
    if (runtime_config_record_flight_files(telemetryFilename, telemetryFilename) != ESP_OK) {
        LOG_ERROR("StorageLoggingTask", "Failed to mark flight log %s as latest", telemetryFilename);
        return false;
    }
    flightLogPrepared = false;
    recordingActive.store(true);
    logger->logInfo("StorageLoggingTask", "Flight recorder active");
    LOG_INFO("StorageLoggingTask", "Flight recorder active: %s", telemetryFilename);
    return true;
}

void StorageLoggingTask::taskFunction() {
    static constexpr uint32_t LOGGER_STATS_PERIOD_MS = 10000;
    static constexpr uint32_t READY_FOR_LAUNCH_BATCH_PERIOD_MS = 5000;
    TickType_t lastWriteTicks = xTaskGetTickCount();
    TickType_t lastStatsTicks = lastWriteTicks;
    TickType_t lastReadyForLaunchBatchTicks = lastWriteTicks;
    uint32_t lastDroppedCount = logger ? logger->getDroppedCount() : 0;
    bool storageWasAvailable = storageInitialized;
    bool wasInReadyForLaunch = false;

    while (running) {
        esp_task_wdt_reset();

        // The task starts in Ground Services, but the FSM opens the file only
        // after locking config. Until then events and recent sensors stay in RAM.
        if (!recordingActive.load() ||
            (fsm && fsm->getCurrentState() == RocketState::GROUND_SERVICES)) {
            vTaskDelay(pdMS_TO_TICKS(80));
            continue;
        }

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

        // Evaluate if we need to trigger a write cycle.
        const int queuedSensorRecords = logger ? logger->getLogCount() : 0;
        TickType_t currentTicks = xTaskGetTickCount();
        bool timeoutReached = (currentTicks - lastWriteTicks) * portTICK_PERIOD_MS >= FLUSH_TIMEOUT_MS;
        const bool inReadyForLaunch = fsm && fsm->getCurrentState() == RocketState::READY_FOR_LAUNCH;
        const bool leftReadyForLaunch = wasInReadyForLaunch && !inReadyForLaunch;
        wasInReadyForLaunch = inReadyForLaunch;
        const bool readyForLaunchBatchDue = inReadyForLaunch &&
            (currentTicks - lastReadyForLaunchBatchTicks) * portTICK_PERIOD_MS >= READY_FOR_LAUNCH_BATCH_PERIOD_MS;

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

        // READY_FOR_LAUNCH writes full batches less often; after it ends, flush
        // the recent pre-launch samples without waiting for the normal threshold.
        const bool eventPending = logger && logger->getEventCount() > 0;
        bool shouldWrite = pendingBytesToWrite > 0;
        if (inReadyForLaunch) {
            shouldWrite = shouldWrite ||
                (readyForLaunchBatchDue && (queuedSensorRecords > 0 || eventPending));
        } else {
            shouldWrite = shouldWrite || eventPending ||
                (queuedSensorRecords > 0 &&
                 (leftReadyForLaunch || timeoutReached || queuedSensorRecords >= BATCH_ENTRY_THRESHOLD));
        }

        // Execute Write
        if (shouldWrite && running && telemetryFilename[0] != '\0') {
            if (pendingBytesToWrite == 0 && logger) {
                pendingBytesToWrite = logger->consumeBatch(writeBuffer, WRITE_BUFFER_SIZE);
                if (inReadyForLaunch) lastReadyForLaunchBatchTicks = currentTicks;
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

void StorageLoggingTask::onTaskStop() {
    if (!recordingActive.load()) return;

    // RECOVERED disables new sensor enqueues before this stop. Drain the
    // existing queues before the recorder task exits and the board reboots.
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(2400);
    while ((pendingBytesToWrite > 0 ||
            (logger && (logger->getLogCount() > 0 || logger->getEventCount() > 0))) &&
           static_cast<int32_t>(deadline - xTaskGetTickCount()) > 0) {
        esp_task_wdt_reset();
        if (!rocketModel || !rocketModel->isStorageInitialized()) break;
        if (pendingBytesToWrite == 0) {
            pendingBytesToWrite = logger->consumeBatch(writeBuffer, WRITE_BUFFER_SIZE);
            if (pendingBytesToWrite == 0) break;
        }
        if (rocketModel->storageAppendFile(telemetryFilename, writeBuffer, pendingBytesToWrite)) {
            pendingBytesToWrite = 0;
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    const bool drained = pendingBytesToWrite == 0 &&
                         (!logger || (logger->getLogCount() == 0 && logger->getEventCount() == 0));
    if (!drained) {
        LOG_ERROR("StorageLoggingTask", "Flight log %s incomplete: pending bytes=%u, queued sensors=%d, events=%d",
                  telemetryFilename, static_cast<unsigned>(pendingBytesToWrite),
                  logger ? logger->getLogCount() : 0, logger ? logger->getEventCount() : 0);
    }
    recordingActive.store(false);
}
