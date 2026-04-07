# Component: I2C
file(GLOB SRCS "*.cpp" "*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES driver
)
