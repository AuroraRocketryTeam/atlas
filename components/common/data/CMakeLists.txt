# Component: data (lib/common/data)
file(GLOB SRCS "src/*.cpp" "src/*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES global
)

# Component example: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-guides/build-system.html#example-of-component-requirements