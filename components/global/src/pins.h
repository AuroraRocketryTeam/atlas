/**
 * @file pins.h
 * @author luca.pulga@studio.unibo.it
 * @brief PIN definition.
 * @version 0.1
 * @date 2024-10-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "hal/gpio_types.h"

// I2C pins
#define I2C_SDA                     GPIO_NUM_11
#define I2C_SCL                     GPIO_NUM_12
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

// SD Card SPI config
#define SD_DET                      -1
#define SD_MAX_TRANSFER_SIZE        4000
#define SD_QUADWP                   -1
#define SD_QUADHD                   -1

// ADC Pin
#define ADC_PIN                     ADC_CHANNEL_0       // From Arduino A0 (GPIO1)

#ifdef CONFIG_ATLAS_MCU_ARDUINO_ESP32

// SD Card pins
#define SD_CLK                      GPIO_NUM_48         // From Arduino D13
#define SD_SO                       GPIO_NUM_47         // From Arduino D12
#define SD_SI                       GPIO_NUM_38         // From Arduino D11
#define SD_CS                       GPIO_NUM_21         // From Arduino D10

// Actuators
#define DROGUE_ACTUATOR_PIN         GPIO_NUM_44         // From Arduino D0
#define MAIN_ACTUATOR_PIN           GPIO_NUM_43         // From Arduino D1
#define BUZZER_PIN                  GPIO_NUM_5          // From Arduino D2

// LED
#define LED_RED_PIN                 GPIO_NUM_3          // From Arduino A2
#define LED_GREEN_PIN               GPIO_NUM_4          // From Arduino A3
#define LED_BLUE_PIN                GPIO_NUM_2          // From Arduino A1

#define GPS_LED                     GPIO_NUM_9          // From Arduino D6

#define ARMING_PIN                  GPIO_NUM_9          // From Arduino D6

// Sensor testing pins
#define IMU_S                       GPIO_NUM_8          // From Arduino D5
#define BAR1_S                      GPIO_NUM_14         // From Arduino A7
#define BAR2_S                      GPIO_NUM_7          // From Arduino D4
#define ACC_S                       GPIO_NUM_13         // From Arduino A6

#elifdef CONFIG_ATLAS_MCU_ESP32S3_WROOM

// SD Card pins.
#define SD_CLK                      GPIO_NUM_17
#define SD_SO                       GPIO_NUM_13
#define SD_SI                       GPIO_NUM_37
#define SD_CS                       GPIO_NUM_10

// Actuators
#define DROGUE_ACTUATOR_PIN         GPIO_NUM_5
#define MAIN_ACTUATOR_PIN           GPIO_NUM_4
#define BUZZER_PIN                  GPIO_NUM_21

// LED
#define LED_RED_PIN                 GPIO_NUM_18
#define LED_GREEN_PIN               GPIO_NUM_8
#define LED_BLUE_PIN                GPIO_NUM_7

#define HOLD_FLASH                  GPIO_NUM_9
#define WP_FLASH                    GPIO_NUM_14

#define CS_BARO                     GPIO_NUM_6

#endif

// Arbitrary builtin LED

#define LED_BUILT_IN LED_BLUE_PIN

// GPS UART pins (TX -> GPS RX, RX -> GPS TX)
#ifndef GPS_TX_PIN
#define GPS_TX_PIN 17
#endif

#ifndef GPS_RX_PIN
#define GPS_RX_PIN 16
#endif

#define LOW        0x0
#define HIGH       0x1

// --- GPIO CONFIGURATIONS ---
const gpio_config_t led_gpio_config = {
    .pin_bit_mask = (1ULL << LED_RED_PIN) | (1ULL << LED_GREEN_PIN) | (1ULL << LED_RED_PIN) | (1ULL << LED_BUILT_IN),
    .mode         = GPIO_MODE_OUTPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE
};

const gpio_config_t actuators_gpio_config = {
    .pin_bit_mask = (1ULL << DROGUE_ACTUATOR_PIN) | (1ULL << MAIN_ACTUATOR_PIN) | (1ULL << BUZZER_PIN),
    .mode         = GPIO_MODE_OUTPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE
};

#ifdef IMU_S
const gpio_config_t sensors_gpio_config = {
    .pin_bit_mask = (1ULL << IMU_S) | (1ULL << BAR1_S) | (1ULL << BAR2_S) | (1ULL << ACC_S),
    .mode         = GPIO_MODE_OUTPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE
};
#endif

#ifdef ARMING_PIN
const gpio_config_t arming_gpio_config = {
    .pin_bit_mask = 1ULL << ARMING_PIN,
    .mode         = GPIO_MODE_INPUT,
    .pull_up_en   = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type    = GPIO_INTR_DISABLE
};
#endif

const i2c_master_bus_config_t i2c_bus0_config = {
    .i2c_port            = I2C_MASTER_NUM,
    .sda_io_num          = I2C_SDA,
    .scl_io_num          = I2C_SCL,
    .clk_source          = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt   = 7,
    .intr_priority       = 0,
    .trans_queue_depth   = 0,
    .flags = { 
        .enable_internal_pullup = false, 
        .allow_pd = false 
    }
};
