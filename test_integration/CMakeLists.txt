cmake_minimum_required(VERSION 2.6)

PROJECT(NTRT_Test_Integration)

SET(ENV_DIR ${PROJECT_SOURCE_DIR}/../env)
SET(ENV_INC_DIR ${ENV_DIR}/include)
SET(ENV_LIB_DIR ${ENV_DIR}/lib)
SET(SRC_DIR ${PROJECT_SOURCE_DIR}/../src)
SET(NTRT_BUILD_DIR ${PROJECT_SOURCE_DIR}/../build)
SET(BULLET_PHYSICS_SOURCE_DIR ${ENV_DIR}/build/bullet)
SET(OPENGL_LIB ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL)
SET(OPENGL_FG_LIB ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL_FreeGlut)

include_directories(${SRC_DIR})

# Env components
include_directories(${ENV_INC_DIR}
					${BULLET_PHYSICS_SOURCE_DIR}/src
					${ENV_INC_DIR}
					${ENV_INC_DIR}/bullet
					${ENV_INC_DIR}/boost
					${ENV_INC_DIR}/tensegrity
					${PROJECT_SOURCE_DIR}
					${SRC_DIR}
					${OPENGL_LIB}
					${OPENGL_FG_LIB}
					"/usr/include/glib-2.0")
link_directories(${ENV_LIB_DIR} ${OPENGL_LIB} ${OPENGL_FG_LIB})

subdirs(
 ICRA2015Tests
 #MuscleNP - Segmentation fault as of 11/26/14
 SpineTests
 TimestepIndependence
 )
