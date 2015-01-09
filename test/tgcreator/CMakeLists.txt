project(tgcreator)

SET(OPENGL_LIB ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL)
SET(OPENGL_FG_LIB ${BULLET_PHYSICS_SOURCE_DIR}/Demos/OpenGL_FreeGlut)
SET(SRC_DIR ${PROJECT_SOURCE_DIR}/../../src)
SET(NTRT_BUILD_DIR ${PROJECT_SOURCE_DIR}/../../build)

include_directories(${CMAKE_CURRENT_BINARY_DIR}
					${ENV_INC_DIR}
					${BULLET_PHYSICS_SOURCE_DIR}/src
					${ENV_INC_DIR}/bullet
					${ENV_INC_DIR}/boost
					${ENV_INC_DIR}/tensegrity
					${SRC_DIR}
					${OPENGL_LIB}
					${OPENGL_FG_LIB})
					
# openGL libs required for core
link_directories(${ENV_LIB_DIR} ${OPENGL_LIB} ${OPENGL_FG_LIB} ${NTRT_BUILD_DIR})


add_executable(tgUtil_test
	tgUtil_test.cpp)

# libcore gets us enough links to bullet to get the btAlignedAllocator
target_link_libraries(tgUtil_test ${ENV_LIB_DIR}/libgtest.a pthread
                        ${NTRT_BUILD_DIR}/core/terrain/libterrain.so
						${NTRT_BUILD_DIR}/core/libcore.so
                        ${NTRT_BUILD_DIR}/tgcreator/libtgcreator.so )
