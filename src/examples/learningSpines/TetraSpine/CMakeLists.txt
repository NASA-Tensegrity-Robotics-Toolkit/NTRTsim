link_directories(${LIB_DIR})

link_libraries(learningSpines
                sensors
                tgcreator
                core
                util
                terrain
                Adapters
                Configuration
                AnnealEvolution
                tgOpenGLSupport)

file(COPY nodeConfig.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY edgeConfig.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_executable(AppTetraSpineLearning
    TetraSpineLearningModel.cpp
    TetraSpineCPGControl.cpp
    AppTetraSpineLearning.cpp
    
) 

add_executable(AppTetraSpineSerial
	SerializedSineWaves.cpp	
    TetraSpineLearningModel.cpp
	AppTetraSpineSerial.cpp
)

target_link_libraries(AppTetraSpineSerial ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
configure_file("controlVars.json" "controlVars.json" COPYONLY)

add_custom_command(TARGET AppTetraSpineLearning POST_BUILD
					COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/logs)
