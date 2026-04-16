# Component: control (lib/control)
file(GLOB_RECURSE SRCS "src/*.cpp" "src/*.c")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src" "src/states" "src/tasks" "src/tasks/hil"
    REQUIRES
        global
        data
        model
        core
        implementation
        SD
        BME680
        BNO055
        GPS
        LIS3DHTR
        MPRLS
        MS561101BA03
        Termoresistenze
        filters_kalman
        status
        protocol
        radio_espnow
        radio_lora
)
