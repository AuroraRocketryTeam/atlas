# Component: Adafruit_Sensor (lib/third_party/Adafruit_Sensor)
file(GLOB SRCS "*.cpp" "*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES espressif__arduino-esp32
)