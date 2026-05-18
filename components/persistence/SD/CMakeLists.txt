# Component: SD (lib/persistence/SD)
file(GLOB SRCS "*.cpp" "*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES storage global data core SPI fatfs sdmmc vfs driver
)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-error=overloaded-virtual")