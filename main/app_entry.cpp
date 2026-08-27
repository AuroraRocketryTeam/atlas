/* 
* @file app_entry.cpp
* @brief Main entry point for the Aurora Rocketry Flight Software.
*
 * This file contains the main entry point for the flight software. It initializes the Arduino framework,
 * sets up the selected mode (flight or HIL simulation), and enters the main loop.
 *
 * The setupSelectedMode() function initializes the appropriate mode based on the configuration, while
 * loopSelectedMode() runs the corresponding loop for the selected mode.
 *
 * The HIL mode selection is splitted in two part to ease the recompilation burder due to the change of the KConfig file.
 * So it's splitted in:
 *  - CONFIG_AURORA_HIL_SUPPORT in KConfig, and
 *  - CONFIG_AURORA_HIL_SIMULATION in config.h
 *
 * @note This project uses the C++14 language standard version.
 */

#include <Arduino.h>
#include <config.h>

void setupFlight();
void loopFlight();

#if CONFIG_AURORA_HIL_SUPPORT
void setupHil();
void loopHil();
#endif

static void setupSelectedMode()
{
#if CONFIG_AURORA_HIL_SIMULATION
    setupHil();
#else
    setupFlight();
#endif
}

static void loopSelectedMode()
{
#if CONFIG_AURORA_HIL_SIMULATION
    loopHil();
#else
    loopFlight();
#endif
}

extern "C" void app_main()
{
    initArduino();

    setupSelectedMode();

    while (true)
    {
        loopSelectedMode();
    }
}
