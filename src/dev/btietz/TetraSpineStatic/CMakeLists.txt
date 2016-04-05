Project(tetraSpineHardware)

link_directories(${LIB_DIR})

link_libraries(learningSpines
                sensors
                tgcreator
                core
                util
                terrain
                tgOpenGLSupport)

add_library(${PROJECT_NAME} SHARED
			TetraSpineStaticModel_hf.cpp
            TetraSpineStaticModel.cpp
            SerializedSpineControl.cpp
            )

add_executable(AppTetraSpineStatic	
	SerializedSpineControl.cpp	
    TetraSpineStaticModel.cpp
    TetraSpineStaticModel_hf.cpp
	AppTetraSpineStatic.cpp
)

target_link_libraries(AppTetraSpineStatic ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
target_link_libraries(tetraSpineHardware ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)

