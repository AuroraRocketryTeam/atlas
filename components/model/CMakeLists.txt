# Component: model (lib/model)
# Sources are at the root (no src/ subdirectory)
file(GLOB SRCS "*.cpp" "*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
    REQUIRES global data
)
