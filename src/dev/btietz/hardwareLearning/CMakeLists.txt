link_directories(${LIB_DIR})

link_libraries(tetraSpineHardware
				learningSpines
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

add_executable(AppTetraSpineHardware
	tgCPGStringControl_mod.cpp
	LearningSpineJSON.cpp
	AppTetraSpineHardwareLearning.cpp
)

target_link_libraries(AppTetraSpineHardware ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
configure_file("controlVars.json" "controlVars.json" COPYONLY)
file(COPY nodeConfig.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY edgeConfig.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})


add_custom_command(TARGET AppTetraSpineHardware POST_BUILD
					COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/logs)
