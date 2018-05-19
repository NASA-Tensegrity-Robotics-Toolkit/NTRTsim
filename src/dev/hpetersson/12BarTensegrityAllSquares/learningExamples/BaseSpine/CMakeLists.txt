Project(learningSpines)

link_libraries(sensors
                tgcreator
                core
                util
                terrain
                Adapters
                Configuration
                AnnealEvolution
                FileHelpers
                tgOpenGLSupport)

add_library(${PROJECT_NAME} SHARED
			BaseSpineModelLearning.cpp
			BaseSpineCPGControl.cpp
            tgCPGActuatorControl.cpp    
            tgCPGLogger.cpp
            tgSCASineControl.cpp
            KinematicSpineCPGControl.cpp
            tgCPGCableControl.cpp
            SpineFeedbackControl.cpp
            )

subdirs(    OctahedralComplex
    ribDemo
    TetrahedralComplex
    TetraSpine
    )
