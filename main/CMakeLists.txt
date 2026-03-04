file(GLOB_RECURSE MAIN_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
                             "${CMAKE_CURRENT_SOURCE_DIR}/*.c")
# register the component to the build system.
idf_component_register(
    SRCS ${MAIN_SRCS}
    INCLUDE_DIRS "."
    PRIV_REQUIRES
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
        Termoresistenze
        filters_kalman
        protocol
        radio_espnow
        radio_lora
        # Third-party
        BNO055_SensorAPI
        RadioLib-master
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