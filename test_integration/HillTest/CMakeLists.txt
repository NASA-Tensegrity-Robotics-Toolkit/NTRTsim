link_directories(${ENV_LIB_DIR} ${NTRT_BUILD_DIR})

link_libraries(
                tgOpenGLSupport)
             
add_executable(TetraSpineHills_test
	TetraSpineHills_test.cpp)

target_link_libraries(TetraSpineHills_test ${ENV_LIB_DIR}/libgtest.a pthread
                                            ${ENV_LIB_DIR}/libjsoncpp.a
                                                ${NTRT_BUILD_DIR}/core/terrain/libterrain.so 
                                                ${NTRT_BUILD_DIR}/core/libcore.so 
                                                ${NTRT_BUILD_DIR}/tgcreator/libtgcreator.so 
                                                ${NTRT_BUILD_DIR}/helpers/libFileHelpers.so 
												${NTRT_BUILD_DIR}/sensors/libsensors.so 
												${NTRT_BUILD_DIR}/examples/learningSpines/liblearningSpines.so 
												${NTRT_BUILD_DIR}/dev/btietz/TetraSpineStatic/libtetraSpineHardware.so
                                                ${NTRT_BUILD_DIR}/dev/btietz/hardwareSineWaves/libtetraSpineLearningSine.so
												${NTRT_BUILD_DIR}/models/obstacles/libobstacles.so
												${NTRT_BUILD_DIR}/dev/btietz/tetraCollisions/libtetraCollisions.so)
												
