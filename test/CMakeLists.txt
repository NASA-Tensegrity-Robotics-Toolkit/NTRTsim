cmake_minimum_required(VERSION 2.6)

PROJECT(NTRT_Test)

SET(ENV_DIR ${PROJECT_SOURCE_DIR}/../env)
SET(ENV_INC_DIR ${ENV_DIR}/include)
SET(ENV_LIB_DIR ${ENV_DIR}/lib)
SET(SRC_DIR ${PROJECT_SOURCE_DIR}/../src)

include_directories(${SRC_DIR})

# Env components
include_directories(${ENV_INC_DIR})
link_directories(${ENV_LIB_DIR})

subdirs(
 helpers)
