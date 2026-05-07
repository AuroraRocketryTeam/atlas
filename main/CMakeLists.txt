set(MAIN_SRCS)

if(CONFIG_AURORA_HIL_SIMULATION)
    list(APPEND MAIN_SRCS "main_hil.cpp")
else()
    list(APPEND MAIN_SRCS "main.cpp")
endif()

# register the component to the build system.
idf_component_register(
    SRCS ${MAIN_SRCS}
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        nvs_flash
        esp_driver_usb_serial_jtag
        global
        data
        control
        model
        core
        implementation
        status
        SD
        BME680
        BNO055
        GPS
        LIS3DHTR
        MPRLS
        MS561101BA03
        I2C
        SPI
        Termoresistenze
        filters_kalman
        protocol
        radio_espnow
        radio_lora
        # Third-party
        # BNO055_SensorAPI
        RadioLib-master
    REQUIRES espressif__arduino-esp32
)


# Note: by default, all the components are compiled, but if we want just some ("data" in this case):
# idf_component_register(
#     SRCS ${app_sources}
#     REQUIRES data
# )
# while, if we want to exclude some components ("data" in this case):
# idf_component_register(
#     SRCS ${app_sources}
#     EXCLUDE data
# )