idf_component_register(
    SRCS "main/nmea_parser.c"
    INCLUDE_DIRS "main"
    KCONFIG "Kconfig"
    REQUIRES esp_event driver
)
