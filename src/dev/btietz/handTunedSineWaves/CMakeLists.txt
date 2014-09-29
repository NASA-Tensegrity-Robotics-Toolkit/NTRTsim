link_directories(${LIB_DIR})


link_libraries(tetraSpineLearningSine
				tetraSpineHardware
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

add_executable(AppTetraSpineHT
	TetraSpineStaticModel_hf.cpp
	htSpineSine.cpp
	AppTetraSpineHT.cpp
)

target_link_libraries(AppTetraSpineHT ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
configure_file("controlVars.json" "controlVars.json" COPYONLY)

