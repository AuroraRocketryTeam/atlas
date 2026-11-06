#include "LEDController.hpp"
#include "SerialLogger.hpp"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "soc/gpio_num.h"

// PWM assignment is BoardHardware, passed via RgbLedcSlots
static constexpr ledc_mode_t LEDC_MODE       = LEDC_LOW_SPEED_MODE;
static constexpr uint32_t    RGB_PWM_FREQ_HZ = 5000;

LEDController::LEDController(gpio_num_t redPin, gpio_num_t greenPin, gpio_num_t bluePin, RgbLedcSlots slots)
    : slots(slots), redPin(redPin), greenPin(greenPin), bluePin(bluePin),
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
    ledc_timer_config_t timer = {};
    timer.speed_mode      = LEDC_MODE;
    timer.duty_resolution = LEDC_TIMER_8_BIT;
    timer.timer_num       = slots.timer;
    timer.freq_hz         = RGB_PWM_FREQ_HZ;
    timer.clk_cfg         = LEDC_AUTO_CLK;
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK)
    {
        LOG_ERROR("LED", "LEDC timer config failed: %s", esp_err_to_name(err));
        return;
    }

    const struct { ledc_channel_t channel; gpio_num_t pin; } channels[] = {
        {slots.red, redPin}, {slots.green, greenPin}, {slots.blue, bluePin}};

    // slots.timer: shared for all of them
    for (const auto &c : channels)
    {
        ledc_channel_config_t cfg = {};
        cfg.gpio_num   = c.pin;
        cfg.speed_mode = LEDC_MODE;
        cfg.channel    = c.channel;
        cfg.intr_type  = LEDC_INTR_DISABLE;
        cfg.timer_sel  = slots.timer;
        cfg.duty       = 0;
        cfg.hpoint     = 0;
        err = ledc_channel_config(&cfg);
        if (err != ESP_OK)
        {
            LOG_ERROR("LED", "LEDC channel %d config failed: %s", c.channel, esp_err_to_name(err));
            return;
        }
    }
    initialized = true;

    // Initialize any status LEDs
    for (uint8_t i = 0; i < statusLedCount; i++)
    {
        gpio_set_level(statusLedPins[i], LOW); // All off
    }

    setOff();
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
    uint8_t r, g, b;
    getRGBValues(color, r, g, b);
    setRGB(r, g, b);
}

void LEDController::setOff()
{
    setRGB(0, 0, 0);
}

void LEDController::setRGB(uint8_t r, uint8_t g, uint8_t b)
{
    if (!initialized) return;

    setChannelDuty(slots.red, r);
    setChannelDuty(slots.green, g);
    setChannelDuty(slots.blue, b);
}

void LEDController::setChannelDuty(ledc_channel_t channel, uint8_t duty)
{
    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
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
