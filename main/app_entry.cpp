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
 * AURORA_HIL_ENABLED is set in the root CMakeLists.txt. It controls both
 * compilation and activation of HIL, so a flight build contains no HIL code.
 *
 * @note This project uses the C++14 language standard version.
 */

#include <Arduino.h>

void setupFlight();
void loopFlight();

#if AURORA_HIL_ENABLED
void setupHil();
void loopHil();
#endif

static void setupSelectedMode()
{
#if AURORA_HIL_ENABLED
    setupHil();
#else
    setupFlight();
#endif
}

static void loopSelectedMode()
{
#if AURORA_HIL_ENABLED
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
