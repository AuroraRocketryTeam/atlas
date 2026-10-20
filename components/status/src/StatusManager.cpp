#include "StatusManager.hpp"
#include <Arduino.h>

StatusManager::StatusManager(LEDController &ledController, BuzzerController &buzzerController)
    : ledController(ledController), buzzerController(buzzerController),
      currentCode(UNKNOWN_ERROR), statusTaskHandle(nullptr)
{
}

void StatusManager::init()
{
    // Initialize pattern map with all patterns from specification
    codeToPattern = {
        // Standard status patterns
        {SYSTEM_OK, {{ART_LED_GREEN, ART_LED_OFF, 0, 0, 0, 0}, BUZZER_SUCCESS, TONE_SUCCESS, false}},
        {PRE_FLIGHT_MODE, {{ART_LED_BLUE, ART_LED_OFF, 0, 0, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {CALIBRATING, {{ART_LED_YELLOW, ART_LED_OFF, 1000, 1000, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {WAITING_INPUT, {{ART_LED_CYAN, ART_LED_OFF, 200, 200, 0, 0}, BUZZER_SHORT_BEEP, TONE_MID, false}},
        {FLIGHT_MODE, {{ART_LED_GREEN, ART_LED_OFF, 0, 0, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {FSM_STARTED, {{ART_LED_ORANGE, ART_LED_OFF, 500, 500, 0, 0}, BUZZER_CONTINUOUS, TONE_MID, true}},

        // Sensor hardware failures
        {IMU_FAIL, {{ART_LED_ORANGE, ART_LED_OFF, 200, 200, 0, 0}, BUZZER_SHORT_BEEP, TONE_ERROR, true}},
        {IMU_DATA_INVALID, {{ART_LED_ORANGE, ART_LED_OFF, 200, 200, 2, 0}, BUZZER_DOUBLE_BEEP, TONE_ERROR, false}},
        {BARO1_FAIL, {{ART_LED_ORANGE, ART_LED_OFF, 200, 200, 0, 0}, BUZZER_SHORT_BEEP, TONE_ERROR, true}},
        {BARO1_DATA_INVALID, {{ART_LED_ORANGE, ART_LED_OFF, 200, 200, 2, 0}, BUZZER_DOUBLE_BEEP, TONE_ERROR, false}},
        {BARO2_FAIL, {{ART_LED_ORANGE, ART_LED_OFF, 800, 400, 0, 0}, BUZZER_SHORT_BEEP, TONE_ERROR, true}},
        {BARO2_DATA_INVALID, {{ART_LED_ORANGE, ART_LED_OFF, 800, 400, 2, 0}, BUZZER_DOUBLE_BEEP, TONE_ERROR, false}},
        {GPS_NO_SIGNAL, {{ART_LED_ORANGE, ART_LED_OFF, 200, 200, 0, 0}, BUZZER_SHORT_BEEP, TONE_ERROR, true}},
        {GPS_DATA_INVALID, {{ART_LED_ORANGE, ART_LED_OFF, 200, 200, 2, 0}, BUZZER_DOUBLE_BEEP, TONE_ERROR, false}},

        // SD card errors
        {SD_MOUNT_FAIL, {{ART_LED_ORANGE, ART_LED_OFF, 300, 300, 3, 0}, BUZZER_WARNING, TONE_ERROR, false}},
        {SD_WRITE_FAIL, {{ART_LED_ORANGE, ART_LED_OFF, 800, 400, 0, 0}, BUZZER_SHORT_BEEP, TONE_ERROR, true}},
        {SD_READ_FAIL, {{ART_LED_ORANGE, ART_LED_OFF, 200, 200, 0, 0}, BUZZER_SHORT_BEEP, TONE_ERROR, true}},

        // // LoRa errors
        // {LORA_INIT_FAIL, {{ART_LED_RED, ART_LED_YELLOW, 400, 400, 0, 0}, BUZZER_WARNING, TONE_ERROR, false}},
        // {LORA_CONFIG_FAIL, {{ART_LED_RED, ART_LED_CYAN, 400, 400, 0, 0}, BUZZER_WARNING, TONE_ERROR, false}},
        // {LORA_TX_FAIL, {{ART_LED_RED, ART_LED_BLUE, 400, 400, 0, 0}, BUZZER_WARNING, TONE_ERROR, false}},

        // // System errors
        // {MEMORY_ERROR, {{ART_LED_YELLOW, ART_LED_RED, 300, 300, 0, 0}, BUZZER_SOS, TONE_ERROR, false}},
        // {TASK_FAIL, {{ART_LED_YELLOW, ART_LED_OFF, 200, 200, 3, 0}, BUZZER_TRIPLE_BEEP, TONE_ERROR, false}},
        // {MUTEX_ERROR, {{ART_LED_YELLOW, ART_LED_OFF, 300, 300, 4, 0}, BUZZER_WARNING, TONE_ERROR, false}},

        // // Power and unknown errors
        // {POWER_LOW_WARNING, {{ART_LED_YELLOW, ART_LED_OFF, 200, 200, 0, 0}, BUZZER_WARNING, TONE_ERROR, true}},
        // {UNKNOWN_ERROR, {{ART_LED_MAGENTA, ART_LED_OFF, 200, 200, 0, 0}, BUZZER_SOS, TONE_ERROR, false}},

        // Test mode specific patterns
        {TEST_MENU, {{ART_LED_BLUE, ART_LED_OFF, 0, 0, 0, 0}, BUZZER_OFF, TONE_OFF, false}},                 // In the test menu
        {TEST_SUCCESS, {{ART_LED_GREEN, ART_LED_OFF, 200, 200, 3, 0}, BUZZER_SUCCESS, TONE_SUCCESS, false}},  // Passed
        {TEST_FAILURE, {{ART_LED_RED, ART_LED_OFF, 200, 200, 3, 0}, BUZZER_ERROR, TONE_ERROR, false}}         // Failed
    };

    initFlightPhases();
}

void StatusManager::initFlightPhases()
{
    phaseToPattern = {
        // Flight phases. The LED is what the pad crew reads from a distance, so the
        // phases that happen on the ground are the ones that have to be
        // unmistakable: white as soon as the board is powered, yellow while it
        // calibrates, cyan once ground services are up, and blinking red the
        // moment it is armed. The buzzer stays out of it: a sounder that starts
        // by itself on the pad is the kind of surprise nobody wants.
        {RocketState::INACTIVE, {{ART_LED_WHITE, ART_LED_OFF, 0, 0, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {RocketState::CALIBRATING, {{ART_LED_YELLOW, ART_LED_OFF, 500, 500, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {RocketState::GROUND_SERVICES, {{ART_LED_CYAN, ART_LED_OFF, 0, 0, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {RocketState::READY_FOR_LAUNCH, {{ART_LED_RED, ART_LED_OFF, 250, 250, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {RocketState::LAUNCH, {{ART_LED_RED, ART_LED_OFF, 0, 0, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {RocketState::ACCELERATED_FLIGHT, {{ART_LED_MAGENTA, ART_LED_OFF, 0, 0, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {RocketState::BALLISTIC_FLIGHT, {{ART_LED_BLUE, ART_LED_OFF, 0, 0, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {RocketState::APOGEE, {{ART_LED_WHITE, ART_LED_OFF, 100, 100, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {RocketState::STABILIZATION, {{ART_LED_YELLOW, ART_LED_OFF, 0, 0, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {RocketState::DECELERATION, {{ART_LED_CYAN, ART_LED_OFF, 150, 150, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {RocketState::LANDING, {{ART_LED_GREEN, ART_LED_OFF, 500, 500, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
        {RocketState::RECOVERED, {{ART_LED_GREEN, ART_LED_OFF, 0, 0, 0, 0}, BUZZER_OFF, TONE_OFF, false}},
    };
}

void StatusManager::setSystemCode(SystemCode code)
{
    // Stop current patterns
    stopAllPatterns();

    // Update current code
    currentCode = code;
    // We are showing a temporary code, so the LED update check
    // will see that currentPhase is empty and the next phase check
    // will repaint. Otherwise the systemCode would be shown until the
    // next phase change.
    currentPhase.reset();

    // Find pattern for this code
    if (codeToPattern.find(code) == codeToPattern.end())
    {
        return; // Pattern not found
    }

    StatusPattern pattern = codeToPattern[code];

    // Special cases that need task-based coordination
    if (pattern.buzzerSync)
    {
        // Create task context for coordinated patterns
        TaskContext *ctx = new TaskContext{
            .instance = this,
            .pattern = pattern};

        // Start coordination task
        xTaskCreatePinnedToCore(
            statusTaskFunction,
            "StatusTask",
            2048,
            ctx,
            1,
            &statusTaskHandle,
            1 // Run on core 1
        );
    }
    else
    {
        applyPattern(pattern);
    }
}

void StatusManager::applyPattern(const StatusPattern &pattern)
{
    ledController.startPattern(pattern.ledPattern);
    buzzerController.startPattern(pattern.buzzerPattern, pattern.buzzerTone);
}

void StatusManager::setFlightPhase(RocketState phase)
{
    if (currentPhase == phase) return;

    stopAllPatterns();
    currentPhase = phase;

    const auto entry = phaseToPattern.find(phase);
    if (entry == phaseToPattern.end()) return;

    applyPattern(entry->second);
}

void StatusManager::stopAllPatterns()
{
    // Stop synchronized task if running
    if (statusTaskHandle != nullptr)
    {
        vTaskDelete(statusTaskHandle);
        statusTaskHandle = nullptr;
    }

    // Stop individual controllers
    ledController.stopPattern();
    buzzerController.stopPattern();
}

const StatusPattern &StatusManager::getPattern(SystemCode code) const
{
    static StatusPattern empty{};
    auto it = codeToPattern.find(code);
    if (it != codeToPattern.end())
        return it->second;
    return empty;
}

uint32_t StatusManager::getPatternDuration(SystemCode code) const
{
    const StatusPattern &pattern = getPattern(code);

    if (pattern.ledPattern.times > 0)
    {
        // Fixed number of blinks
        return (pattern.ledPattern.duration + pattern.ledPattern.pause) * pattern.ledPattern.times + 200;
    }
    else if (pattern.ledPattern.duration == 0 && pattern.ledPattern.pause == 0)
    {
        // Solid color
        return 0; // Infinite
    }
    else
    {
        // Continuous blinking - default to showing at least a few cycles
        return (pattern.ledPattern.duration + pattern.ledPattern.pause) * 3;
    }
}

void StatusManager::playBlockingPattern(SystemCode code, uint32_t minDuration)
{
    // Set the pattern
    setSystemCode(code);

    // Get pattern duration
    uint32_t duration = getPatternDuration(code);

    // If pattern is continuous (duration = 0) or minDuration is specified, use minDuration
    if (duration == 0 || (minDuration > 0 && minDuration > duration))
    {
        duration = minDuration;
    }

    // Wait for pattern to complete
    if (duration > 0)
    {
        vTaskDelay(pdMS_TO_TICKS(duration));
    }
    else
    {
        // For infinite patterns, just show for a reasonable time
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void StatusManager::statusTaskFunction(void *param)
{
    // Extract context
    TaskContext *ctx = static_cast<TaskContext *>(param);
    StatusManager *instance = ctx->instance;
    StatusPattern pattern = ctx->pattern;

    // Free context memory
    delete ctx;

    // Default synchronized behavior
    while (true)
    {
        // LED on
        instance->ledController.setColor(pattern.ledPattern.color1);

        // Buzzer on if synchronized
        if (pattern.buzzerSync)
            instance->buzzerController.playToneFreq(pattern.buzzerTone);

        // Hold for duration
        vTaskDelay(pdMS_TO_TICKS(pattern.ledPattern.duration));

        // LED off
        instance->ledController.setOff();

        // Buzzer off if synchronized
        if (pattern.buzzerSync)
            instance->buzzerController.stopTone();

        // Pause
        vTaskDelay(pdMS_TO_TICKS(pattern.ledPattern.pause));
    }

    // Task is done
    instance->statusTaskHandle = nullptr;
    vTaskDelete(NULL);
}