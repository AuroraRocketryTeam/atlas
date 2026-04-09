#include "LEDController.hpp"
#include "driver/gpio.h"
#include "soc/gpio_num.h"

LEDController::LEDController(gpio_num_t redPin, gpio_num_t greenPin, gpio_num_t bluePin)
    : redPin(redPin), greenPin(greenPin), bluePin(bluePin),
      statusLedCount(0), statusLedPins(nullptr), ledTaskHandle(nullptr)
{
}

LEDController::~LEDController()
{
    stopPattern();

    // Free status LED pin array
    if (statusLedPins != nullptr)
    {
        delete[] statusLedPins;
        statusLedPins = nullptr;
    }
}

void LEDController::addStatusLeds(gpio_num_t *pins, uint8_t count)
{
    // Clean up old pins if any
    if (statusLedPins != nullptr)
    {
        delete[] statusLedPins;
    }

    // Allocate and copy new pins
    statusLedCount = count;
    statusLedPins = new gpio_num_t[count];
    memcpy(statusLedPins, pins, count * sizeof(gpio_num_t));
}

void LEDController::init()
{


    // Initialize any status LEDs
    for (uint8_t i = 0; i < statusLedCount; i++)
    {
        gpio_set_level(statusLedPins[i], LOW); // All off
    }

    // Initialize LED to off state
    setRGB(0, 0, 0);
}

void LEDController::startPattern(const LEDPattern &pattern)
{
    // Stop any current pattern
    stopPattern();

    // Create task context
    TaskContext *ctx = new TaskContext{
        .instance = this,
        .pattern = pattern};

    // Start the LED task
    xTaskCreatePinnedToCore(
        ledTaskFunction,
        "LEDTask",
        2048,
        ctx,
        1,
        &ledTaskHandle,
        1 // Run on core 1
    );
}

void LEDController::stopPattern()
{
    // If there's a running task, delete it
    if (ledTaskHandle != nullptr)
    {
        vTaskDelete(ledTaskHandle);
        ledTaskHandle = nullptr;
    }

    // Turn off all outputs
    setRGB(0, 0, 0);

    // Turn off all status LEDs
    for (uint8_t i = 0; i < statusLedCount; i++)
    {
        gpio_set_level(statusLedPins[i], LOW);
    }
}

void LEDController::getRGBValues(LEDColor color, uint8_t &r, uint8_t &g, uint8_t &b)
{
    switch (color)
    {
    case ART_LED_RED:
        r = 255;
        g = 0;
        b = 0;
        break;
    case ART_LED_GREEN:
        r = 0;
        g = 255;
        b = 0;
        break;
    case ART_LED_BLUE:
        r = 0;
        g = 0;
        b = 255;
        break;
    case ART_LED_YELLOW:
        r = 255;
        g = 255;
        b = 0;
        break;
    case ART_LED_CYAN:
        r = 0;
        g = 255;
        b = 255;
        break;
    case ART_LED_MAGENTA:
        r = 255;
        g = 0;
        b = 255;
        break;
    case ART_LED_WHITE:
        r = 255;
        g = 255;
        b = 255;
        break;
    case ART_LED_ORANGE:
        r = 255;
        g = 128;
        b = 0;
        break;
    case ART_LED_OFF:
    default:
        r = 0;
        g = 0;
        b = 0;
        break;
    }
}

void LEDController::setColor(LEDColor color)
{
    switch (color) {
        case ART_LED_RED:     setRGB(1, 0, 0); break;
        case ART_LED_GREEN:   setRGB(0, 1, 0); break;
        case ART_LED_BLUE:    setRGB(0, 0, 1); break;
        case ART_LED_YELLOW:  setRGB(1, 1, 0); break;
        case ART_LED_CYAN:    setRGB(0, 1, 1); break;
        case ART_LED_MAGENTA: setRGB(1, 0, 1); break;
        case ART_LED_WHITE:   setRGB(1, 1, 1); break;
        case ART_LED_ORANGE:  setRGB(1, 1, 0); break; // no PWM, yellow approximation
        case ART_LED_OFF:
        default:              setRGB(0, 0, 0); break;
    }
}

void LEDController::setOff()
{
    setRGB(0, 0, 0);
}

void LEDController::setRGB(uint8_t r, uint8_t g, uint8_t b)
{
    gpio_set_level(redPin,   r > 0 ? 1 : 0);
    gpio_set_level(greenPin, g > 0 ? 1 : 0);
    gpio_set_level(bluePin,  b > 0 ? 1 : 0);
}

void LEDController::setStatusLeds(uint8_t mask)
{
    // Set each status LED based on bitmask
    for (uint8_t i = 0; i < statusLedCount && i < 8; i++)
    {
        gpio_set_level(statusLedPins[i], (mask & (1 << i)) ? HIGH : LOW);
    }
}

void LEDController::showOutputLED(LEDColor color, uint16_t duration_ms, uint16_t pause_ms, uint8_t times)
{
    for (int i = 0; i < times; i++)
    {
        setColor(color);

        // LED on for duration
        vTaskDelay(pdMS_TO_TICKS(duration_ms));

        // Turn off LED
        setOff();

        // Pause between blinks (except last one)
        if (i < times - 1)
        {
            vTaskDelay(pdMS_TO_TICKS(pause_ms));
        }
    }
}

void LEDController::ledTaskFunction(void *param)
{
    // Extract context
    TaskContext *ctx = static_cast<TaskContext *>(param);
    LEDController *instance = ctx->instance;
    LEDPattern pattern = ctx->pattern;

    // Free context memory
    delete ctx;

    // Set auxiliary LEDs if defined
    if (instance->statusLedCount > 0)
    {
        instance->setStatusLeds(pattern.auxLeds);
    }

    // Special case for solid colors
    if (pattern.duration == 0 && pattern.pause == 0 && pattern.times == 0)
    {
        // Set the solid color
        instance->setColor(pattern.color1);

        // Keep task alive for permanent patterns
        while (true)
        {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Default pattern behavior
    while (true)
    {
        // Handle alternating colors
        if (pattern.color2 != ART_LED_OFF)
        {
            instance->showOutputLED(pattern.color1, pattern.duration, 0, 1);
            instance->showOutputLED(pattern.color2, pattern.duration, pattern.pause, 1);
        }
        // Handle repeated blinks (specified by times > 0)
        else if (pattern.times > 0)
        {
            for (int i = 0; i < pattern.times; i++)
            {
                // Turn on RGB LED
                instance->setColor(pattern.color1);

                // LED on for duration
                vTaskDelay(pdMS_TO_TICKS(pattern.duration));

                // Turn off RGB LED
                instance->setOff();

                // Pause between blinks (except last one)
                if (i < pattern.times - 1)
                {
                    vTaskDelay(pdMS_TO_TICKS(pattern.pause));
                }
            }

            // Longer pause after the pattern completes
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        // Handle continuous blinking
        else
        {
            // Turn on RGB LED
            instance->setColor(pattern.color1);

            // LED on for duration
            vTaskDelay(pdMS_TO_TICKS(pattern.duration));

            // Turn off RGB LED
            instance->setOff();

            // Pause
            vTaskDelay(pdMS_TO_TICKS(pattern.pause));
        }

        // Exit if it's a one-time pattern
        if (pattern.times > 0 && pattern.color2 == ART_LED_OFF)
        {
            break;
        }
    }

    // Task is done, exit
    instance->ledTaskHandle = nullptr;
    vTaskDelete(NULL);
}
