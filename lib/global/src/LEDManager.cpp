#include "LEDManager.h"
#include <map>
#include <Arduino.h>
#include <config.h>
#include <pins.h>

// Tabella mapping codice → pattern
static std::map<SystemCode, LedPattern> codeToPattern = {
    {SYSTEM_OK, {ART_LED_GREEN, ART_LED_OFF, 1000, 0, 1}},
    {PRE_FLIGHT_MODE, {ART_LED_BLUE, ART_LED_OFF, 1000, 0, 1}},
    {CALIBRATING, {ART_LED_YELLOW, ART_LED_OFF, 500, 500, 0}},
    {WAITING_INPUT, {ART_LED_CYAN, ART_LED_OFF, 300, 300, 0}},

    {IMU_FAIL, {ART_LED_RED, ART_LED_OFF, 200, 200, 1}},
    {IMU_DATA_INVALID, {ART_LED_RED, ART_LED_OFF, 200, 200, 2}},
    {BARO1_FAIL, {ART_LED_RED, ART_LED_OFF, 200, 200, 3}},
    {BARO1_DATA_INVALID, {ART_LED_RED, ART_LED_OFF, 200, 200, 4}},
    {BARO2_FAIL, {ART_LED_RED, ART_LED_OFF, 200, 200, 5}},
    {BARO2_DATA_INVALID, {ART_LED_RED, ART_LED_OFF, 200, 200, 6}},
    {GPS_NO_SIGNAL, {ART_LED_RED, ART_LED_OFF, 200, 200, 7}},
    {GPS_DATA_INVALID, {ART_LED_RED, ART_LED_OFF, 200, 200, 8}},

    {SD_MOUNT_FAIL, {ART_LED_RED, ART_LED_OFF, 700, 300, 0}},
    {SD_WRITE_FAIL, {ART_LED_RED, ART_LED_OFF, 400, 200, 0}},
    {SD_READ_FAIL, {ART_LED_RED, ART_LED_OFF, 200, 100, 0}},

    {LORA_INIT_FAIL, {ART_LED_RED, ART_LED_BLUE, 500, 500, 0}},
    {LORA_CONFIG_FAIL, {ART_LED_RED, ART_LED_GREEN, 500, 500, 0}},
    {LORA_TX_FAIL, {ART_LED_RED, ART_LED_YELLOW, 500, 500, 0}},

    {MEMORY_ERROR, {ART_LED_YELLOW, ART_LED_OFF, 200, 200, 0}},
    {TASK_FAIL, {ART_LED_YELLOW, ART_LED_OFF, 200, 200, 2}},
    {MUTEX_ERROR, {ART_LED_YELLOW, ART_LED_OFF, 200, 200, 3}},

    {POWER_LOW_WARNING, {ART_LED_YELLOW, ART_LED_OFF, 200, 200, 0}},
    {UNKNOWN_ERROR, {ART_LED_MAGENTA, ART_LED_OFF, 1000, 0, 1}}};

// Task FreeRTOS che gestisce il lampeggio
void ledTask(void *param)
{
    LedPattern pattern = *(LedPattern *)param;
    free(param); // liberiamo la memoria allocata in setSystemLED()
    while (true)
    {
        if (pattern.color2 == ART_LED_OFF)
        {
            showOutputLED(pattern.color1, pattern.duration, pattern.pause, pattern.times == 0 ? 1 : pattern.times);
        }
        else
        {
            showOutputLED(pattern.color1, pattern.duration, 0, 1);
            showOutputLED(pattern.color2, pattern.duration, pattern.pause, 1);
        }
        if (pattern.times > 0)
        {
            break;
        }
    }
    vTaskDelete(NULL);
}

void setSystemLED(SystemCode code)
{
    if (codeToPattern.find(code) == codeToPattern.end())
        return;

    LedPattern *pattern = new LedPattern(codeToPattern[code]); // alloc dinamica
    xTaskCreatePinnedToCore(ledTask, "LEDTask", 2048, pattern, 1, NULL, 1);
}

// Imposta direttamente i canali RGB
inline void setRGB(uint8_t r, uint8_t g, uint8_t b)
{
    analogWrite(LED_RED_PIN, r);
    analogWrite(LED_GREEN_PIN, g);
    analogWrite(LED_BLUE_PIN, b);
}

// Blink LED con colore e pattern
void showOutputLED(LEDColor color, uint16_t duration_ms, uint16_t pause_ms, uint8_t times)
{
    for (int i = 0; i < times; i++)
    {
        switch (color)
        {
        case ART_LED_RED:
            setRGB(255, 0, 0);
            break;
        case ART_LED_GREEN:
            setRGB(0, 255, 0);
            break;
        case ART_LED_BLUE:
            setRGB(0, 0, 255);
            break;
        case ART_LED_YELLOW:
            setRGB(255, 255, 0);
            break;
        case ART_LED_CYAN:
            setRGB(0, 255, 255);
            break;
        case ART_LED_MAGENTA:
            setRGB(255, 0, 255);
            break;
        case ART_LED_WHITE:
            setRGB(255, 255, 255);
            break;
        case ART_LED_OFF:
            setRGB(0, 0, 0);
            break;
        default:
            setRGB(0, 0, 0);
            break;
        }

        // LED acceso per durata
        vTaskDelay(pdMS_TO_TICKS(duration_ms));

        // Spegni LED
        setRGB(0, 0, 0);

        // Pausa tra i lampeggi (eccetto ultimo)
        if (i < times - 1)
        {
            vTaskDelay(pdMS_TO_TICKS(pause_ms));
        }
    }
}
