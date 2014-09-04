project(helpers)

# Compose our header file for resource inclusion
set(RESOURCE_PATH "${CMAKE_SOURCE_DIR}/../resources/src")
configure_file("${helpers_SOURCE_DIR}/resources.h.in" "${helpers_BINARY_DIR}/resources.h")

add_library(FileHelpers SHARED
    FileHelpers.cpp)
