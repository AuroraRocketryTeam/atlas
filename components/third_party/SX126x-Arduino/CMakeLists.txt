# Component: SX126x-Arduino
file(GLOB_RECURSE SRCS "src/*.cpp" "src/*.c")
file(GLOB_RECURSE ALL_HEADERS "src/*.h")
set(INCLUDE_DIRS "src")
foreach(HEADER ${ALL_HEADERS})
    get_filename_component(DIR ${HEADER} DIRECTORY)
    list(APPEND INCLUDE_DIRS ${DIR})
endforeach()
list(REMOVE_DUPLICATES INCLUDE_DIRS)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES espressif__arduino-esp32
)