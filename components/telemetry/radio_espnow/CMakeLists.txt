# Component: radio_espnow (lib/telemetry/radio_espnow)
file(GLOB SRCS "src/*.cpp" "src/*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES global data protocol core
)
