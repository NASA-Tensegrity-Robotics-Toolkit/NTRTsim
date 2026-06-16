Project(tetraSpineLearningSine)

link_directories(${LIB_DIR})

add_library(${PROJECT_NAME} SHARED
            tgSineStringControl.cpp
            )

target_link_libraries(${PROJECT_NAME}
    tetraSpineHardware
    learningSpines
    sensors
    tgcreator
    util
    controllers
    core
    terrain
    Adapters
    Configuration
    AnnealEvolution
    tgOpenGLSupport)

link_libraries(tetraSpineHardware
		learningSpines
                sensors
                tgcreator
                util
                controllers
                core
                terrain
                Adapters
                Configuration
                AnnealEvolution
                tgOpenGLSupport)

add_executable(AppTetraSpineHardwareSine
	tgSineStringControl.cpp
	LearningSpineSine.cpp
	AppTetraSpineHardwareSine.cpp
)

ntrt_add_runme(AppTetraSpineHardwareSine DESCRIPTION "Run the hardware sine waves demo")
