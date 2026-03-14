# Component: EByte_LoRa_E220_Series_Library
file(GLOB SRCS "*.cpp" "*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES espressif__arduino-esp32
)