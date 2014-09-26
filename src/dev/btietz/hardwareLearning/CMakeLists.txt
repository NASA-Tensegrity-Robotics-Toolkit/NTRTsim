link_directories(${LIB_DIR})

link_libraries(tetraSpineHardware
				learningSpines
                sensors
                tgcreator
                core
                util
                terrain
                Adapters
                Configuration
                AnnealEvolution
                tgOpenGLSupport)

add_executable(AppTetraSpineHardware
	LearningSpineJSON.cpp
	AppTetraSpineHardwareLearning.cpp
)

target_link_libraries(AppTetraSpineHardware ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
configure_file("controlVars.json" "controlVars.json" COPYONLY)


