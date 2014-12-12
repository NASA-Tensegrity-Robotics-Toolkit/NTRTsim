link_directories(${LIB_DIR})

link_libraries(KinematicString
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

add_library(TetrahedralComplex SHARED
	FlemonsSpineModelLearning.cpp)
   

add_executable(AppFlemonsSpineLearning
    FlemonsSpineModelLearning.cpp
    AppFlemonsSpineLearning.cpp
    
) 
