link_directories(${ENV_LIB_DIR} ${NTRT_BUILD_DIR})

link_libraries(
                tgOpenGLSupport)
             
add_executable(WorldConf_Spines_test
	WorldConf_Spines_test.cpp)

target_link_libraries(WorldConf_Spines_test ${ENV_LIB_DIR}/libgtest.a pthread 
												${NTRT_BUILD_DIR}/core/libcore.so 
												${NTRT_BUILD_DIR}/helpers/libFileHelpers.so 
												${NTRT_BUILD_DIR}/examples/learningSpines/liblearningSpines.so 
												${NTRT_BUILD_DIR}/dev/btietz/kinematicString/libKinematicString.so 
												${NTRT_BUILD_DIR}/examples/learningSpines/TetrahedralComplex/libTetrahedralComplex.so)
