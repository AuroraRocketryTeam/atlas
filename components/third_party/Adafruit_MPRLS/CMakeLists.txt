# Component: Adafruit_MPRLS (lib/third_party/Adafruit_MPRLS)
file(GLOB SRCS "*.cpp" "*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES espressif__arduino-esp32 Adafruit_BusIO
)