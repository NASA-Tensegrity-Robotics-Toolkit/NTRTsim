cmake_minimum_required(VERSION 2.6)

PROJECT(NTRT_Test)

include_directories("../src/")

# Env components
include_directories(../env/include/)
link_directories(../env/lib/)

SET(ENV_DIR ${PROJECT_SOURCE_DIR}/../env)
SET(ENV_INC_DIR ${ENV_DIR}/include)
SET(ENV_LIB_DIR ${ENV_DIR}/lib)

subdirs(
 helpers)
