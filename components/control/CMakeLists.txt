# Component: control (lib/control)
file(GLOB_RECURSE SRCS "src/*.cpp" "src/*.c")

set(INCLUDE_DIRS
    "src"
    "src/states"
    "src/tasks"
)

# HIL sources are picked up by GLOB_RECURSE. Keep them only for HIL builds.
if(CONFIG_AURORA_HIL_SIMULATION)
    list(APPEND INCLUDE_DIRS "src/tasks/hil")
else()
    list(FILTER SRCS EXCLUDE REGEX ".*/src/tasks/hil/.*")
endif()

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES
        global
        data
        model
        rocket_logger
        SD
        BNO055
        GPS
        LIS3DHTR
        MS561101BA03
        status
        protocol
        radio_espnow
        radio_lora
)
