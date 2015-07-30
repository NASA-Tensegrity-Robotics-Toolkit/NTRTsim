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

OPTION(USE_DOUBLE_PRECISION "Use double precision"  ON)

IF (USE_DOUBLE_PRECISION)
ADD_DEFINITIONS( -DBT_USE_DOUBLE_PRECISION)
SET( BULLET_DOUBLE_DEF "-DBT_USE_DOUBLE_PRECISION")
ENDIF (USE_DOUBLE_PRECISION)

subdirs(
 helpers
 tgcreator
 util)
