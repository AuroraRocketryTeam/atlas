# Component: global (lib/global)
# Header-only component (config, pins, telemetry fields)
# espressif__arduino-esp32 is added here so all downstream components
# that REQUIRE global get Arduino.h access transitively.
idf_component_register(
    INCLUDE_DIRS "src"
    REQUIRES nlohmann_json espressif__arduino-esp32
)
