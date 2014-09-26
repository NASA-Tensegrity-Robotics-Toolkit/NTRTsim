Project(tetraSpineHardware)

link_directories(${LIB_DIR})

add_library(${PROJECT_NAME} SHARED
   
            TetraSpineStaticModel.cpp
            SerializedSpineControl.cpp
            )

link_libraries(learningSpines
                sensors
                tgcreator
                core
                util
                terrain
                tgOpenGLSupport)

add_executable(AppTetraSpineStatic
	SerializedStaticControl.cpp	
	SerializedSpineControl.cpp	
    TetraSpineStaticModel.cpp
	AppTetraSpineStatic.cpp
)

target_link_libraries(AppTetraSpineStatic ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
target_link_libraries(tetraSpineHardware ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
configure_file("controlVars.json" "controlVars.json" COPYONLY)
configure_file("controlVarsStatic.json" "controlVarsStatic.json" COPYONLY)

