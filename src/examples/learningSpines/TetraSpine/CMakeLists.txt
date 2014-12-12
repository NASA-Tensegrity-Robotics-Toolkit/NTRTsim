link_directories(${LIB_DIR})

link_libraries(learningSpines
                sensors
                tgcreator
                controllers
                core
                util
                terrain
                Adapters
                Configuration
                AnnealEvolution
                tgOpenGLSupport)

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

