cmake_minimum_required(VERSION 2.6)

PROJECT(NTRT_Test)

include_directories("../src/")

# Env components
include_directories(../env/include/)
link_directories(../env/lib/)

subdirs(
 helpers)
