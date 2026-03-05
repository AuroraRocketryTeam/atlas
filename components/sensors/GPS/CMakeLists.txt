# Component: GPS (lib/sensors/GPS)
file(GLOB SRCS "src/*.cpp" "src/*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES global data SparkFun_u-blox_GNSS_Arduino_Library-2.2.27
)
