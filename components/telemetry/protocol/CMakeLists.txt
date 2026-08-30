# Component: protocol (lib/telemetry/protocol)
file(GLOB SRCS "src/*.cpp" "src/*.c")

# WHOLE_ARCHIVE: radio_lora calls Packet::calculateCRC()/printPacket() from this
# component. Whether the linker resolves those depends on where ESP-IDF places
# libprotocol.a relative to libradio_lora.a, and that order shifts whenever the
# component graph changes. Linking the whole archive makes it order-independent;
# the archive is two small objects.
idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "src"
    REQUIRES global data
    WHOLE_ARCHIVE
)
