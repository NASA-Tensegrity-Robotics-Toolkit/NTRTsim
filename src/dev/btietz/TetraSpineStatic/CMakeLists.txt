link_directories(${LIB_DIR})

link_libraries(learningSpines
                sensors
                tgcreator
                core
                util
                terrain
                tgOpenGLSupport)

add_executable(AppTetraSpineStatic
	SerializedSpineControl.cpp	
    TetraSpineStaticModel.cpp
	AppTetraSpineStatic.cpp
)

target_link_libraries(AppTetraSpineStatic ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
configure_file("controlVars.json" "controlVars.json" COPYONLY)

