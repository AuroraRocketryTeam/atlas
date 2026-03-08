#include "Arduino.h"

// Example of an old basic Arduino setup/loop to test the ESP-IDF integration. 
// This file MUST be replaced by the actual main logic in main_true.cpp, but for now serves as a sanity check that the Arduino core is working correctly.
void setup() {
    Serial.begin(115200);
    Serial.println("Arduino is running inside ESP-IDF!");
}

void loop() {
    Serial.println("Looping...");
    delay(1000);
}

// The ESP-IDF entry point, which must be C-linkage
extern "C" void app_main() {
    // Initialize the Arduino core background tasks
    initArduino();
    
    setup();
    
    while (1) {
        loop();
        // Using a FreeRTOS task delay instead of an empty while(1) to prevent the watchdog timer from triggering.    
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}