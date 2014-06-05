Project(btietz)

link_libraries(sensors
                tgcreator
                core
                util
                terrain
                Adapters
                Configuration
                AnnealEvolution
                tgOpenGLSupport)

add_library(${PROJECT_NAME} SHARED
			BaseSpineModelLearning.cpp
			BaseSpineCPGControl.cpp
            tgCPGStringControl.cpp    
            tgRBString.cpp
            tgRBStringInfo.cpp
            )

subdirs(    learningTests
    learningTestsCrossLink
    learningTestsTetra
    RBTests
    ribDemo
    )
