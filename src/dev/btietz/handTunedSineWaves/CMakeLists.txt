link_directories(${LIB_DIR})


link_libraries(tetraSpineLearningSine
				tetraSpineHardware
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

add_executable(AppTetraSpineHT
	htSpineSine.cpp
	AppTetraSpineHT.cpp
)

target_link_libraries(AppTetraSpineHT ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
configure_file("controlVars.json" "controlVars.json" COPYONLY)

