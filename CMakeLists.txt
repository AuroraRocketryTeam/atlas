cmake_minimum_required(VERSION 3.16)

# Development build profile. ON compiles and starts the HIL simulator; OFF
# omits all HIL code from the flight firmware. Edit this value, then build.
set(AURORA_HIL_ENABLED OFF)

# Each entry must point directly to a component folder (one that has a CMakeLists.txt)
set(EXTRA_COMPONENT_DIRS
    # common
    ${CMAKE_SOURCE_DIR}/components/common/data

    # top-level components
    ${CMAKE_SOURCE_DIR}/components/control
    ${CMAKE_SOURCE_DIR}/components/global
    ${CMAKE_SOURCE_DIR}/components/model
    ${CMAKE_SOURCE_DIR}/components/status

    # sensors
    ${CMAKE_SOURCE_DIR}/components/sensors/BNO055
    ${CMAKE_SOURCE_DIR}/components/sensors/GPS
    ${CMAKE_SOURCE_DIR}/components/sensors/LIS3DHTR
    ${CMAKE_SOURCE_DIR}/components/sensors/MS561101BA03

    # telemetry
    ${CMAKE_SOURCE_DIR}/components/telemetry/protocol
    ${CMAKE_SOURCE_DIR}/components/telemetry/radio_espnow
    ${CMAKE_SOURCE_DIR}/components/telemetry/radio_lora

    # logger
    ${CMAKE_SOURCE_DIR}/components/logger/rocket_logger
    ${CMAKE_SOURCE_DIR}/components/logger/serializations

    # persistence
    ${CMAKE_SOURCE_DIR}/components/persistence/storage
    ${CMAKE_SOURCE_DIR}/components/persistence/SD
    ${CMAKE_SOURCE_DIR}/components/persistence/flash

    # protocols
    ${CMAKE_SOURCE_DIR}/components/protocols/I2C
    ${CMAKE_SOURCE_DIR}/components/protocols/SPI

    # third_party
    ${CMAKE_SOURCE_DIR}/components/third_party/nlohmann_json
    ${CMAKE_SOURCE_DIR}/components/third_party/ArduinoEigen
    ${CMAKE_SOURCE_DIR}/components/third_party/BNO055_SensorAPI
    ${CMAKE_SOURCE_DIR}/components/third_party/RadioLib-master
    ${CMAKE_SOURCE_DIR}/components/third_party/SdFat
    ${CMAKE_SOURCE_DIR}/components/third_party/lis3dh_pid
    ${CMAKE_SOURCE_DIR}/components/third_party/TinyEKF-master
    ${CMAKE_SOURCE_DIR}/components/third_party/Adafruit_BusIO
    ${CMAKE_SOURCE_DIR}/components/third_party/Adafruit_Sensor
    ${CMAKE_SOURCE_DIR}/components/third_party/EByte_LoRa_E220_Series_Library
    ${CMAKE_SOURCE_DIR}/components/third_party/SX126x-Arduino
    ${CMAKE_SOURCE_DIR}/components/third_party/ArxTypeTraits
    ${CMAKE_SOURCE_DIR}/components/third_party/nmea0183_parser
)

# Ensure a placeholder Kconfig exists so kconfgen does not fail when no project Kconfig is generated
if(NOT EXISTS "${CMAKE_BINARY_DIR}/Kconfig")
    file(WRITE "${CMAKE_BINARY_DIR}/Kconfig" "# Auto-generated placeholder for project Kconfig\n")
endif()

set(SDKCONFIG_DEFAULTS "${CMAKE_SOURCE_DIR}/sdkconfig.defaults")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# This registers all the components in the specified directory, so IT IS IMPORTANT to put all the EXTRA_COMPONENT_DIRS before this
# Note: if two components have the same name the highest priority one will override the other: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/build-system.html#multiple-components-with-the-same-name
project(atlas)

# Relax -Werror for implicit-function-declaration — needed for Arduino-ESP32 3.x
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=implicit-function-declaration" APPEND)

# To add compilation flags:
# idf_build_set_property(COMPILE_OPTIONS "-Wno-error" APPEND)
