set(MAIN_SRCS "app_entry.cpp" "main.cpp")

if(CONFIG_AURORA_HIL_SUPPORT)
    list(APPEND MAIN_SRCS "main_hil.cpp")
endif()

# register the component to the build system.
idf_component_register(
    SRCS ${MAIN_SRCS}
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        nvs_flash
        spi_flash
        esp_hw_support
        app_update
        esp_driver_usb_serial_jtag
        global
        data
        control
        model
        rocket_logger
        serializations
        status
        SD
        flash
        BNO055
        GPS
        LIS3DHTR
        MS561101BA03
        I2C
        SPI
        protocol
        radio_lora
        tests
        # Third-party
        BNO055_SensorAPI
        RadioLib-master
    REQUIRES espressif__arduino-esp32
)

# TODO
# The serializer is header-only and instantiated by these two entry points.
# Keep their object files dependent on its implementation headers even if
# generated compiler depfiles are unavailable.
set_source_files_properties(main.cpp main_hil.cpp PROPERTIES OBJECT_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/../components/logger/serializations/src/PayloadSerializer.hpp;${CMAKE_CURRENT_LIST_DIR}/../components/logger/rocket_logger/src/LogPayload.hpp")


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
