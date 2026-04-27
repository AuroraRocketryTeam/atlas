idf_component_register(
    SRCS "Flash.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver global SPI SD spi_flash esp_partition vfs joltwallet__littlefs
)
