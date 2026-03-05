# Component: SD (lib/persistence/SD)
file(GLOB SRCS "src/*.cpp" "src/*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES global data SdFat core
)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-error=overloaded-virtual")