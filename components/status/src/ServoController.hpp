#pragma once
#include "IAirbrakesController.hpp"
#include "soc/gpio_num.h"
#include <driver/ledc.h>
#include <cstdint>

class ServoController : public IAirbrakesController
{
public:
    /**
     * @brief Construct the controller
     * @param signalPin GPIO for control signalk
     * @param timer LEDC timer reserved for the servo
     * @param channel LEDC channel reserved for the servo
     * @param powerPin GPIO held high as power source, GPIO_NUM_NC for none
     * @param servo_freq Frequency at which the servo can operate (~signal resolution)
     */
    ServoController(gpio_num_t signalPin, ledc_timer_t timer, ledc_channel_t channel,
                    uint32_t closedUs, uint32_t openUs,
                    gpio_num_t powerPin = GPIO_NUM_NC, uint32_t servo_freq = 100);

    /**
     * @brief Init the controller and power on the powerPin
     *
     * @return true if timer and channel successfully configured
     */
    bool start() override;

    /**
     * @brief Stop the pulses and release the signal and power pins.
     */
    void stop() override;

    /**
     * @brief Command servo to deployment
     * @param deployment 0.0f closed, 1.0f open.
     */
    void command(float deployment) override;

    /**
     * @brief Set the pulse width
     * @param pulseUs High time of each pulse in microseconds
     */
    void writeMicroseconds(uint32_t pulseUs);

    bool isStarted() const { return _started; }
    uint32_t getFrequency() const { return _freq; }
    bool setFrequency(uint32_t freq);

private:
    gpio_num_t     _signalPin;
    ledc_timer_t   _timer;
    ledc_channel_t _channel;
    gpio_num_t     _powerPin;
    bool           _started;
    uint32_t       _freq;
    uint32_t       _closed_us;
    uint32_t       _open_us;
};
