# Component: BNO055_SensorAPI (lib/third_party/BNO055_SensorAPI)
file(GLOB SRCS "src/*.c" "src/*.cpp")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
)
