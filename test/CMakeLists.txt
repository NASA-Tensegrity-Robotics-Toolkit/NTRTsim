cmake_minimum_required(VERSION 2.6)

PROJECT(NTRT_Test)

subdirs(
 core)

add_subdirectory("../src/" "../build_test/")

link_directories(${LIB_DIR})

include_directories("../src/")
