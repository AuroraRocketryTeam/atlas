# Component: control (lib/control)
file(GLOB_RECURSE SRCS "src/*.cpp" "src/*.c")

set(SDKCONFIG_SNAPSHOT "${CMAKE_CURRENT_BINARY_DIR}/embedded_sdkconfig.txt")
if(EXISTS "${CMAKE_SOURCE_DIR}/sdkconfig")
    configure_file("${CMAKE_SOURCE_DIR}/sdkconfig" "${SDKCONFIG_SNAPSHOT}" COPYONLY)
else()
    file(WRITE "${SDKCONFIG_SNAPSHOT}" "sdkconfig was not available when CMake configured this build. Re-run the build after sdkconfig is generated.\n")
endif()

set(INCLUDE_DIRS
    "src"
    "src/states"
    "src/tasks"
)

# HIL sources are picked up by GLOB_RECURSE. Keep them only for HIL builds.
if(CONFIG_AURORA_HIL_SUPPORT)
    list(APPEND INCLUDE_DIRS "src/tasks/hil")
else()
    list(FILTER SRCS EXCLUDE REGEX ".*/src/tasks/hil/.*")
endif()

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    EMBED_TXTFILES
        # "src/tasks/certs/servercert.pem"
        # "src/tasks/certs/prvtkey.pem"
        "src/tasks/web/index.html"
        "src/tasks/web/style.css"
        "src/tasks/web/app.js"
        "${SDKCONFIG_SNAPSHOT}"
    REQUIRES
        app_update
        esp_app_format
        esp_event
        # esp_https_server
        esp_http_server
        esp_netif
        esp_timer
        esp_wifi
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
        nvs_flash
        spi_flash
)
