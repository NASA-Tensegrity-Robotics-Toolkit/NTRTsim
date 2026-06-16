link_directories(${LIB_DIR})

link_libraries(learningSpines
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

ntrt_add_runme(AppFlemonsSpineLearning DESCRIPTION "Run the Flemons spine learning demo")
