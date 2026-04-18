file(GLOB_RECURSE SRCS "src/*.cpp")

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES
        global
        data
        model
        core
        implementation
        control
        status
        SD
        flash
        I2C
        radio_lora
    REQUIRES espressif__arduino-esp32
)
