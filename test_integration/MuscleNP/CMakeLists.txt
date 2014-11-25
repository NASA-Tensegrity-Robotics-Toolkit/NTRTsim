link_directories(${ENV_LIB_DIR} ${NTRT_BUILD_DIR})

link_libraries( tgOpenGLSupport
                )
             
add_executable(MuscleNP_test
	MuscleNP_test.cpp)

target_link_libraries(MuscleNP_test ${ENV_LIB_DIR}/libgtest.a pthread 
												${NTRT_BUILD_DIR}/core/libcore.so 
												${NTRT_BUILD_DIR}/core/terrain/libterrain.so 
												${NTRT_BUILD_DIR}/tgcreator/libtgcreator.so
												${NTRT_BUILD_DIR}/dev/muscleNP/libMuscleNP.so
												${NTRT_BUILD_DIR}/dev/btietz/muscleNPEnergy/libMuscleNPCons.so
												 )
