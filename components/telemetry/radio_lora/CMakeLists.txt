# Component: radio_lora (lib/telemetry/radio_lora)
file(GLOB SRCS "src/*.cpp" "src/*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES global data protocol RadioLib-master EByte_LoRa_E220_Series_Library
)
