file(GLOB SRCS "*.cpp" "*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES global data rocket_logger
)
