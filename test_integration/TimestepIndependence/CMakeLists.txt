link_directories(${ENV_LIB_DIR} ${NTRT_BUILD_DIR})

link_libraries(
                tgOpenGLSupport)
             
add_executable(MotorTimestep_test
	MotorTimestep_test.cpp)

target_link_libraries(MotorTimestep_test ${ENV_LIB_DIR}/libgtest.a pthread 
			${NTRT_BUILD_DIR}/core/libcore.so
			${NTRT_BUILD_DIR}/dev/btietz/kinematicString/libKinematicString.so
			${NTRT_BUILD_DIR}/dev/btietz/timestepTest/libTimestepTest.so)
