#pragma once
#include "soc/gpio_num.h"
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstdint>

// Buzzer tones (frequencies in Hz)
enum BuzzerTone
{
    TONE_OFF = 0,
    TONE_LOW = 220,    // Low A
    TONE_MID = 440,    // Middle A
    TONE_HIGH = 880,   // High A
    TONE_ALERT = 1760, // Very high A
    TONE_ERROR = 330,  // E - creates dissonance for error sounds
    TONE_SUCCESS = 587 // D - pleasant for success sounds
};

// Buzzer patterns
enum BuzzerPattern
{
    BUZZER_OFF,
    BUZZER_CONTINUOUS,  // Constant tone
    BUZZER_SHORT_BEEP,  // Single short beep
    BUZZER_DOUBLE_BEEP, // Two short beeps
    BUZZER_TRIPLE_BEEP, // Three short beeps
    BUZZER_SOS,         // SOS pattern (... --- ...)
    BUZZER_WARNING,     // Alternating high/low tone
    BUZZER_INTERMITTENT, // Even on/off beeping, 50% duty
    BUZZER_COUNTDOWN,   // Descending tones
    BUZZER_SUCCESS,     // Rising tone sequence
    BUZZER_ERROR        // Error pattern
};

// Buzzer Pattern Definition
struct BuzzerSequence
{
    BuzzerTone tone;   // Frequency
    uint16_t duration; // Duration in ms
    uint16_t pause;    // Pause after the tone
};

namespace Note
{
    constexpr uint16_t REST = 0;
    constexpr uint16_t C3 =  131, CS3 =  139, D3 =  147, DS3 =  156, E3 =  165, F3 =  175, FS3 =  185, G3 =  196, GS3 =  208, A3 =  220, AS3 =  233, B3 =  247;
    constexpr uint16_t C4 =  262, CS4 =  277, D4 =  294, DS4 =  311, E4 =  330, F4 =  349, FS4 =  370, G4 =  392, GS4 =  415, A4 =  440, AS4 =  466, B4 =  494;
    constexpr uint16_t C5 =  523, CS5 =  554, D5 =  587, DS5 =  622, E5 =  659, F5 =  698, FS5 =  740, G5 =  784, GS5 =  831, A5 =  880, AS5 =  932, B5 =  988;
    constexpr uint16_t C6 = 1047, CS6 = 1109, D6 = 1175, DS6 = 1245, E6 = 1319, F6 = 1397, FS6 = 1480, G6 = 1568, GS6 = 1661, A6 = 1760, AS6 = 1865, B6 = 1976;
}

struct MelodyNote
{
    uint16_t frequency;
    uint16_t duration; 
};

/**
 * @brief Class to control a buzzer with various patterns and tones.
 * 
 */
class BuzzerController
{
private:
    // Buzzer pin
    gpio_num_t buzzerPin;

    // The LEDC timer and channel this board reserved for the buzzer. A timer of
    // its own is not a luxury: setting a note means setting the frequency.
    ledc_timer_t timer;
    ledc_channel_t channel;
    bool initialized = false;

    // Current active pattern
    TaskHandle_t buzzerTaskHandle;

    // Internal task context structure
    struct TaskContext
    {
        BuzzerController *instance;
        BuzzerPattern pattern;
        BuzzerTone baseTone; // Base tone for the pattern
    };

    // Static task function for FreeRTOS
    static void buzzerTaskFunction(void *param);
    static void melodyTaskFunction(void *param);

    // What the melody task repeats. Only one melody task exists at a time, so
    // these live here rather than in a heap context the task cannot free when
    // stopPattern() deletes it.
    const MelodyNote *_loopNotes = nullptr;
    uint8_t _loopCount = 0;
    uint16_t _loopGapMs = 0;
    uint16_t _loopPauseMs = 0;

    // Melody player without the stopPattern(), so the melody task does not
    // delete itself halfway through a tune.
    void playMelodyNotes(const MelodyNote *notes, uint8_t count, uint16_t gapMs);

    // Internal pattern implementation
    void playPatternInternal(BuzzerPattern pattern, BuzzerTone baseTone = TONE_MID);

public:
    // Constructor & Destructor
    BuzzerController(gpio_num_t buzzerPin, ledc_timer_t timer, ledc_channel_t channel);
    ~BuzzerController();

    // Initialize pins
    void init();

    // Control buzzer
    void startPattern(BuzzerPattern pattern, BuzzerTone baseTone = TONE_MID);
    void stopPattern();

    // Simple direct controls
    void playTone(BuzzerTone frequency, uint16_t duration);
    void playSequence(const BuzzerSequence *sequence, uint8_t count, bool repeat = false);
    void setOff();

    /**
     * @brief Play a melody, blocking the calling task until it ends.
     *
     * Any running pattern is stopped first.
     */
    void playMelody(const MelodyNote *notes, uint8_t count, uint16_t gapMs = 20);

    /** @brief Play the built in jingle. */
    void playJingle();

    /**
     * @brief Repeat a melody in a task of its own until stopPattern() is called.
     */
    void startMelodyLoop(const MelodyNote *notes, uint8_t count, uint16_t gapMs = 20, uint16_t pauseMs = 1500);

    /** @brief Repeat the built in jingle until stopPattern() is called. */
    void startJingleLoop(uint16_t pauseMs = 1500);

    // Play a single tone
    void playToneFreq(uint16_t frequency);
    void stopTone();
};
