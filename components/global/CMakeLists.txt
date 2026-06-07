# Component: global (lib/global)
# Header-only component (config, pins, telemetry fields)
# espressif__arduino-esp32 is added here so all downstream components
# that REQUIRE global get Arduino.h access transitively.

file(GLOB SRCS "src/*.cpp" "src/*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES nlohmann_json espressif__arduino-esp32 esp_driver_usb_serial_jtag
)
