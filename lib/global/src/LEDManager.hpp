#pragma once
#include <Arduino.h>

// Available colors
enum LEDColor
{
    ART_LED_OFF,
    ART_LED_RED,
    ART_LED_GREEN,
    ART_LED_BLUE,
    ART_LED_YELLOW,
    ART_LED_CYAN,
    ART_LED_MAGENTA,
    ART_LED_WHITE
};

// Codici di stato / errore
enum SystemCode
{
    SYSTEM_OK = 0,
    PRE_FLIGHT_MODE,
    CALIBRATING,
    WAITING_INPUT,

    IMU_FAIL = 20,
    IMU_DATA_INVALID,
    BARO1_FAIL,
    BARO1_DATA_INVALID,
    BARO2_FAIL,
    BARO2_DATA_INVALID,
    GPS_NO_SIGNAL,
    GPS_DATA_INVALID,

    SD_MOUNT_FAIL = 40,
    SD_WRITE_FAIL,
    SD_READ_FAIL,

    LORA_INIT_FAIL = 50,
    LORA_CONFIG_FAIL,
    LORA_TX_FAIL,

    MEMORY_ERROR = 80,
    TASK_FAIL,
    MUTEX_ERROR,

    POWER_LOW_WARNING = 90,
    UNKNOWN_ERROR = 99
};

// Pattern LED
struct LedPattern
{
    LEDColor color1;
    LEDColor color2;   // per alternanza
    uint16_t duration; // ms accensione
    uint16_t pause;    // ms di pausa
    uint8_t times;     // numero lampeggi (0 = continuo)
};

// API principale
void setSystemLED(SystemCode code);

// Devi implementare questa funzione nel tuo progetto (driver fisico del LED)
void showOutputLED(LEDColor color, uint16_t duration, uint16_t pause, uint8_t times);
