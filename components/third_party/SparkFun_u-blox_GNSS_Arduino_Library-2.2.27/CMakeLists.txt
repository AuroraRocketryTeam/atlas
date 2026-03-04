# Component: SparkFun_u-blox_GNSS_Arduino_Library-2.2.27 (lib/third_party)
file(GLOB SRCS "src/*.cpp" "src/*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
)
