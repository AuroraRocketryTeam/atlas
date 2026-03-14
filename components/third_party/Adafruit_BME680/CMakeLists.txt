# Component: Adafruit_BME680 (lib/third_party/Adafruit_BME680)
file(GLOB SRCS "*.cpp" "*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES espressif__arduino-esp32 Adafruit_BusIO Adafruit_Sensor
)