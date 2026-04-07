# Component: model (lib/model)
# Sources are at the root (no src/ subdirectory)
file(GLOB_RECURSE SRCS "*.cpp" "*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "." "boards_hardware" "boards_hardware/manny"
    REQUIRES global data BNO055 MPRLS LIS3DHTR MS561101BA03 GPS Termoresistenze core implementation
)
