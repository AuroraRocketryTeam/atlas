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

/**
 * @brief I2C STD PIN.
 *
 */
// Pin I2C.
#define I2C_SDA 11
#define I2C_SCL 12
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

// SD Card pins.
#define SD_CLK 21// from Arduino D13
#define SD_SO 38// from Arduino D12
#define SD_SI 47// from Arduino D11
#define SD_CS 48// from Arduino D10
#define SD_DET -1
#define SD_MAX_TRANSFER_SIZE 4000
#define SD_QUADWP -1
#define SD_QUADHD -1

// Actuators
#define DROGUE_ACTUATOR_PIN 5 // Replaced D0 with actual pin number 5
#define MAIN_ACTUATOR_PIN 4  // Replaced D1 with actual pin number 4
#define BUZZER_PIN 6         // Replaced D2 with actual pin number 6

// LED
#define LED_RED_PIN A2
#define LED_GREEN_PIN A3
#define LED_BLUE_PIN A1

#define GPS_LED 7            // Replaced D6 with actual pin number 7

#define ARMING_PIN 7         // Replaced D6 with actual pin number 7

