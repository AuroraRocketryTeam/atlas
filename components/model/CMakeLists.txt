# Component: model (lib/model)
file(GLOB SRCS "RocketModel.cpp" "*.cpp" "*.c")

if(CONFIG_COMPILE_BOARD_MANNY)
    list(APPEND SRCS "boards_hardware/manny/board.cpp")
    set(BOARD_INCLUDE_DIRS "boards_hardware/manny")
else()
    # Don't fail if we're just generating the menuconfig
    set(BOARD_INCLUDE_DIRS "")
endif()

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "." "boards_hardware" ${BOARD_INCLUDE_DIRS}
    REQUIRES global data I2C SPI SD flash BNO055 MPRLS LIS3DHTR MS561101BA03 GPS Termoresistenze core implementation
)
