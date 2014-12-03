Project(tetraSpineHardware)

link_directories(${LIB_DIR})

add_library(${PROJECT_NAME} SHARED
			TetraSpineStaticModel_hf.cpp
            TetraSpineStaticModel.cpp
            SerializedSpineControl.cpp
            )

link_libraries(FileHelpers
				${ENV_LIB_DIR}/libjsoncpp.a
				learningSpines
                sensors
                tgcreator
                core
                util
                terrain
                tgOpenGLSupport)

add_executable(AppTetraSpineStatic	
	SerializedSpineControl.cpp	
    TetraSpineStaticModel.cpp
    TetraSpineStaticModel_hf.cpp
	AppTetraSpineStatic.cpp
)
