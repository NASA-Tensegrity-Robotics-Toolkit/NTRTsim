Project(learningSpines)

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
            )

subdirs(    OctahedralComplex
    ribDemo
    TetrahedralComplex
    TetraSpine
    )
