#include "BuzzerController.hpp"
#include "SerialLogger.hpp"
#include <driver/ledc.h>

static constexpr ledc_mode_t      LEDC_MODE            = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_bit_t TONE_DUTY_RESOLUTION = LEDC_TIMER_10_BIT;
// maximum volume is half of the range (10 bit), which is 512.
static constexpr uint32_t         TONE_DUTY_HALF       = 512;

BuzzerController::BuzzerController(gpio_num_t buzzerPin, ledc_timer_t timer, ledc_channel_t channel)
    : buzzerPin(buzzerPin), timer(timer), channel(channel), buzzerTaskHandle(nullptr)
{
}

BuzzerController::~BuzzerController()
{
    stopPattern();
}

void BuzzerController::init()
{
    if (buzzerPin <= 0) return;

    ledc_timer_config_t timerConfig = {};
    timerConfig.speed_mode      = LEDC_MODE;
    timerConfig.duty_resolution = TONE_DUTY_RESOLUTION;
    timerConfig.timer_num       = timer;
    timerConfig.freq_hz         = TONE_MID;
    timerConfig.clk_cfg         = LEDC_AUTO_CLK;
    if (ledc_timer_config(&timerConfig) != ESP_OK)
    {
        LOG_ERROR("Buzzer", "LEDC timer config failed");
        return;
    }

    ledc_channel_config_t channelConfig = {};
    channelConfig.gpio_num   = buzzerPin;
    channelConfig.speed_mode = LEDC_MODE;
    channelConfig.channel    = channel;
    channelConfig.intr_type  = LEDC_INTR_DISABLE;
    channelConfig.timer_sel  = timer;
    channelConfig.duty       = 0;
    channelConfig.hpoint     = 0;
    if (ledc_channel_config(&channelConfig) != ESP_OK)
    {
        LOG_ERROR("Buzzer", "LEDC channel config failed");
        return;
    }

    initialized = true;
    stopTone();
}

void BuzzerController::startPattern(BuzzerPattern pattern, BuzzerTone baseTone)
{
    // If no buzzer, skip
    if (buzzerPin == 0)
        return;

    // Stop any current pattern
    stopPattern();

    // For simple patterns, play directly
    if (pattern == BUZZER_OFF || pattern == BUZZER_SHORT_BEEP ||
        pattern == BUZZER_DOUBLE_BEEP || pattern == BUZZER_TRIPLE_BEEP)
    {
        playPatternInternal(pattern, baseTone);
        return;
    }

    // For continuous patterns, create a task
    TaskContext *ctx = new TaskContext{
        .instance = this,
        .pattern = pattern,
        .baseTone = baseTone};

    // Start the buzzer task
    xTaskCreatePinnedToCore(
        buzzerTaskFunction,
        "BuzzerTask",
        2048,
        ctx,
        1,
        &buzzerTaskHandle,
        1 // Run on core 1
    );
}

void BuzzerController::stopPattern()
{
    // If there's a running task, delete it
    if (buzzerTaskHandle != nullptr)
    {
        vTaskDelete(buzzerTaskHandle);
        buzzerTaskHandle = nullptr;
    }

    // Turn off buzzer
    stopTone();
}

void BuzzerController::playToneFreq(uint16_t frequency)
{
    if (!initialized || frequency == 0) return;

    // The note is the frequency of the timer
    ledc_set_freq(LEDC_MODE, timer, frequency);
    ledc_set_duty(LEDC_MODE, channel, TONE_DUTY_HALF);
    ledc_update_duty(LEDC_MODE, channel);
}

void BuzzerController::stopTone()
{
    if (!initialized) return;

    ledc_set_duty(LEDC_MODE, channel, 0);
    ledc_update_duty(LEDC_MODE, channel);
}

static const MelodyNote JINGLE[] = {
    {Note::E4, 415}, {Note::E4, 415}, {Note::GS4, 197}, {Note::FS4, 197}, {Note::E4, 415},
    {Note::GS4, 415}, {Note::GS4, 415}, {Note::B4, 197}, {Note::A4, 197}, {Note::GS4, 415},
    {Note::A4, 415}, {Note::A4, 415}, {Note::C5, 197}, {Note::B4, 197}, {Note::A4, 415},
};

void BuzzerController::playMelodyNotes(const MelodyNote *notes, uint8_t count, uint16_t gapMs)
{
    for (uint8_t i = 0; i < count; i++)
    {
        // A rest is a note with no frequency
        if (notes[i].frequency > 0) playToneFreq(notes[i].frequency);
        else stopTone();

        vTaskDelay(pdMS_TO_TICKS(notes[i].duration));
        stopTone();

        if (gapMs > 0) vTaskDelay(pdMS_TO_TICKS(gapMs));
    }
}

void BuzzerController::playMelody(const MelodyNote *notes, uint8_t count, uint16_t gapMs)
{
    if (!initialized || notes == nullptr) return;

    stopPattern();
    playMelodyNotes(notes, count, gapMs);
}

void BuzzerController::playJingle()
{
    playMelody(JINGLE, sizeof(JINGLE) / sizeof(JINGLE[0]));
}

void BuzzerController::melodyTaskFunction(void *param)
{
    BuzzerController *self = static_cast<BuzzerController *>(param);

    while (true)
    {
        self->playMelodyNotes(self->_loopNotes, self->_loopCount, self->_loopGapMs);
        self->stopTone();
        vTaskDelay(pdMS_TO_TICKS(self->_loopPauseMs));
    }
}

void BuzzerController::startMelodyLoop(const MelodyNote *notes, uint8_t count, uint16_t gapMs, uint16_t pauseMs)
{
    if (!initialized || notes == nullptr || count == 0) return;

    stopPattern();

    _loopNotes = notes;
    _loopCount = count;
    _loopGapMs = gapMs;
    _loopPauseMs = pauseMs;

    // Low priority
    xTaskCreatePinnedToCore(melodyTaskFunction, "BuzzerMelody", 2048, this, 1, &buzzerTaskHandle, 1);
}

void BuzzerController::startJingleLoop(uint16_t pauseMs)
{
    startMelodyLoop(JINGLE, sizeof(JINGLE) / sizeof(JINGLE[0]), 20, pauseMs);
}

void BuzzerController::playTone(BuzzerTone frequency, uint16_t duration)
{
    if (buzzerPin == 0 || frequency == TONE_OFF)
        return;

    playToneFreq(frequency);
    vTaskDelay(pdMS_TO_TICKS(duration));
    stopTone();
}

void BuzzerController::playSequence(const BuzzerSequence *sequence, uint8_t count, bool repeat)
{
    if (buzzerPin == 0 || sequence == nullptr || count == 0)
        return;

    do
    {
        for (uint8_t i = 0; i < count; i++)
        {
            if (sequence[i].tone != TONE_OFF)
            {
                playToneFreq(sequence[i].tone);
                vTaskDelay(pdMS_TO_TICKS(sequence[i].duration));
                stopTone();
            }
            if (sequence[i].pause > 0)
            {
                vTaskDelay(pdMS_TO_TICKS(sequence[i].pause));
            }
        }
    } while (repeat);
}

void BuzzerController::setOff()
{
    // Stop any ongoing tone
    stopTone();
}

void BuzzerController::playPatternInternal(BuzzerPattern pattern, BuzzerTone baseTone)
{
    if (buzzerPin == 0)
        return;

    switch (pattern)
    {
    case BUZZER_CONTINUOUS:
        playToneFreq(baseTone);
        break;

    case BUZZER_SHORT_BEEP:
        playTone(baseTone, 100);
        break;

    case BUZZER_DOUBLE_BEEP:
    {
        BuzzerSequence seq[] = {
            {baseTone, 100, 100},
            {baseTone, 100, 0}};
        playSequence(seq, 2);
        break;
    }

    case BUZZER_TRIPLE_BEEP:
    {
        BuzzerSequence seq[] = {
            {baseTone, 100, 100},
            {baseTone, 100, 100},
            {baseTone, 100, 0}};
        playSequence(seq, 3);
        break;
    }

    case BUZZER_SOS:
    {
        // ... --- ...
        BuzzerSequence seq[] = {
            // 3 short beeps (...)
            {baseTone, 200, 200},
            {baseTone, 200, 200},
            {baseTone, 200, 400},
            // 3 long beeps (---)
            {baseTone, 600, 200},
            {baseTone, 600, 200},
            {baseTone, 600, 400},
            // 3 short beeps (...)
            {baseTone, 200, 200},
            {baseTone, 200, 200},
            {baseTone, 200, 0}};
        playSequence(seq, 9);
        break;
    }

    case BUZZER_WARNING:
    {
        BuzzerSequence seq[] = {
            // High tones (faster beeps)
            {TONE_HIGH, 100, 100},
            {TONE_HIGH, 100, 100},
            {TONE_HIGH, 100, 300},
            // Low tones (slower beeps)
            {TONE_LOW, 300, 200},
            {TONE_LOW, 300, 400}};
        playSequence(seq, 5, true); // Repeat the warning pattern
        break;
    }

    case BUZZER_COUNTDOWN:
    {
        // Descending tones
        BuzzerSequence seq[] = {
            {TONE_HIGH, 500, 300},
            {(BuzzerTone)(TONE_HIGH * 7 / 8), 500, 300},
            {(BuzzerTone)(TONE_HIGH * 6 / 8), 500, 300},
            {(BuzzerTone)(TONE_HIGH * 5 / 8), 500, 300},
            {(BuzzerTone)(TONE_HIGH * 4 / 8), 1000, 0}};
        playSequence(seq, 5);
        break;
    }

    case BUZZER_SUCCESS:
    {
        // Ascending tones
        BuzzerSequence seq[] = {
            {TONE_LOW, 100, 50},
            {(BuzzerTone)(TONE_LOW * 5 / 4), 100, 50},
            {(BuzzerTone)(TONE_LOW * 6 / 4), 100, 50},
            {(BuzzerTone)(TONE_LOW * 8 / 4), 200, 0}};
        playSequence(seq, 4);
        break;
    }

    case BUZZER_ERROR:
    {
        // Falling error tones
        BuzzerSequence seq[] = {
            {TONE_HIGH, 200, 50},
            {TONE_ERROR, 400, 0}};
        playSequence(seq, 2);
        break;
    }

    case BUZZER_OFF:
    default:
        stopTone();
        break;
    }
}

void BuzzerController::buzzerTaskFunction(void *param)
{
    // Extract context
    TaskContext *ctx = static_cast<TaskContext *>(param);
    BuzzerController *instance = ctx->instance;
    BuzzerPattern pattern = ctx->pattern;
    BuzzerTone baseTone = ctx->baseTone;

    // Free context memory
    delete ctx;

    // For continuous buzzer
    if (pattern == BUZZER_CONTINUOUS)
    {
        instance->playToneFreq(baseTone);
        while (true)
        {
            vTaskDelay(pdMS_TO_TICKS(1000)); // Just keep the task alive
        }
    }
    // For repeating patterns
    else
    {
        while (true)
        {
            instance->playPatternInternal(pattern, baseTone);
            vTaskDelay(pdMS_TO_TICKS(2000)); // Pause between pattern repetitions
        }
    }

    // Should never reach here, but cleanup just in case
    instance->buzzerTaskHandle = nullptr;
    vTaskDelete(NULL);
}
