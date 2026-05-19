# Component: BNO055 (lib/sensors/BNO055)
file(GLOB SRCS "src/*.cpp" "src/*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES global data BNO055_SensorAPI I2C nvs_flash core
)
