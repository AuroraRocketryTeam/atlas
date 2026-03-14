# Component: ArduinoEigen (lib/third_party/ArduinoEigen)
# Header-only Eigen wrapper — just expose the include path
idf_component_register(
    INCLUDE_DIRS "." "ArduinoEigen"
    REQUIRES ArxTypeTraits
)
