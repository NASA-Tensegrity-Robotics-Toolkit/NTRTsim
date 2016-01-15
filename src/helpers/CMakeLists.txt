project(helpers)

include_directories(${CMAKE_CURRENT_BINARY_DIR})

# Compose our header file for resource inclusion
set(RESOURCE_PATH "${CMAKE_SOURCE_DIR}/../resources/src")
set(TEST_RESOURCE_PATH "${CMAKE_SOURCE_DIR}/../resources/test")
configure_file("${helpers_SOURCE_DIR}/resources.h.in" "${helpers_BINARY_DIR}/resources.h")

add_library(FileHelpers SHARED
    FileHelpers.cpp)
