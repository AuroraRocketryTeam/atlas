#include "ServoController.hpp"
#include "SerialLogger.hpp"
#include "driver/ledc.h"
#include <cstdint>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static constexpr ledc_mode_t      LEDC_MODE             = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_bit_t SERVO_DUTY_RESOLUTION = LEDC_TIMER_14_BIT;
// Time between power supply and first pulse
static constexpr uint32_t         POWER_SETTLE_MS       = 300;

ServoController::ServoController(gpio_num_t signalPin, ledc_timer_t timer, ledc_channel_t channel,
                                 uint32_t closedUs, uint32_t openUs,
                                 gpio_num_t powerPin, uint32_t servo_freq)
    : _signalPin(signalPin), _timer(timer), _channel(channel), _powerPin(powerPin), _started(false), _freq(servo_freq), 
      _closed_us(closedUs), _open_us(openUs)
{}

bool ServoController::start()
{
    if (_started) return true;

    if (_powerPin != GPIO_NUM_NC) {
        // Highest drive strength
        gpio_reset_pin(_powerPin);
        gpio_set_level(_powerPin, 1);
        gpio_set_direction(_powerPin, GPIO_MODE_OUTPUT);
        gpio_set_drive_capability(_powerPin, GPIO_DRIVE_CAP_3);
        vTaskDelay(pdMS_TO_TICKS(POWER_SETTLE_MS));
    }

    ledc_timer_config_t timerConfig = {};
    timerConfig.speed_mode      = LEDC_MODE;
    timerConfig.duty_resolution = SERVO_DUTY_RESOLUTION;
    timerConfig.timer_num       = _timer;
    timerConfig.freq_hz         = _freq;
    timerConfig.clk_cfg         = LEDC_AUTO_CLK;
    esp_err_t err = ledc_timer_config(&timerConfig);
    if (err != ESP_OK) {
        LOG_ERROR("Servo", "LEDC timer config failed: %s", esp_err_to_name(err));
        if (_powerPin != GPIO_NUM_NC) gpio_reset_pin(_powerPin);
        return false;
    }

    ledc_channel_config_t channelConfig = {};
    channelConfig.gpio_num   = _signalPin;
    channelConfig.speed_mode = LEDC_MODE;
    channelConfig.channel    = _channel;
    channelConfig.intr_type  = LEDC_INTR_DISABLE;
    channelConfig.timer_sel  = _timer;
    channelConfig.duty       = 0;
    channelConfig.hpoint     = 0;
    err = ledc_channel_config(&channelConfig);
    if (err != ESP_OK) {
        LOG_ERROR("Servo", "LEDC channel config failed: %s", esp_err_to_name(err));
        if (_powerPin != GPIO_NUM_NC) gpio_reset_pin(_powerPin);
        return false;
    }

    _started = true;
    LOG_INFO("Servo", "Servo started at freq %lu", _freq);
    return true;
}

void ServoController::writeMicroseconds(uint32_t pulseUs)
{
    if (!_started) return;

    const uint64_t fullScale = 1ULL << SERVO_DUTY_RESOLUTION;
    const uint32_t duty = static_cast<uint32_t>(pulseUs * fullScale * _freq / 1000000ULL);
    // Consider changing hpoint throughout the various PWMs if we want to reduce power spikes.
    // TODO: single point of configuration hpoint configuration. 
    // ledc_set_duty_and_update(LEDC_MODE, _channel, duty, 0U); Doesn't work.
    ledc_set_duty(LEDC_MODE, _channel, duty);
    ledc_update_duty(LEDC_MODE, _channel);
}

void ServoController::stop()
{
    if (!_started) return;

    ledc_stop(LEDC_MODE, _channel, 0);
    gpio_reset_pin(_signalPin);
    if (_powerPin != GPIO_NUM_NC) gpio_reset_pin(_powerPin);
    _started = false;
}

bool ServoController::setFrequency(uint32_t new_freq) {
    ledc_timer_config_t timerConfig = {};
    timerConfig.speed_mode      = LEDC_MODE;
    timerConfig.duty_resolution = SERVO_DUTY_RESOLUTION;
    timerConfig.timer_num       = _timer;
    timerConfig.freq_hz         = new_freq;
    timerConfig.clk_cfg         = LEDC_AUTO_CLK;
    esp_err_t err = ledc_timer_config(&timerConfig);
    if (err != ESP_OK) {
        LOG_ERROR("Servo", "LEDC timer config failed: %s", esp_err_to_name(err));
        if (_powerPin != GPIO_NUM_NC) gpio_reset_pin(_powerPin);
        return false;
    }
    _freq = new_freq;
    return true;
}

/**
    * @brief Map a normalized deployment to a servo pulse width.
    * @param deployment 0.0f fully closed, 1.0f fully open.
    * @return Pulse width in microseconds, linear between the two ends.
    */
void ServoController::command(float deployment)
{
    if (deployment < 0.0f || deployment > 1.0f) {
        LOG_WARNING("Servo", "Commanded impossible deployment %f!", deployment);
    };
    
    writeMicroseconds(_closed_us + deployment * (_open_us - _closed_us));
}
