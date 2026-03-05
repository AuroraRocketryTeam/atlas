# Component: Seeed_Arduino_LIS3DHTR-1.2.4 (lib/third_party/Seeed_Arduino_LIS3DHTR-1.2.4)
file(GLOB SRCS "src/*.cpp" "src/*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES espressif__arduino-esp32
)
