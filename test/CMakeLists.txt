cmake_minimum_required(VERSION 2.6)

PROJECT(NTRT_Test)

add_subdirectory("../src/", "../build_test/")

subdirs(
    core
    )
