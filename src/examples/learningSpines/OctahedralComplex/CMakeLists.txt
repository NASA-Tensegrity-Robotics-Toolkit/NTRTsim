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

add_library (octahedralComplex SHARED
				FlemonsSpineModelLearningCL.cpp)

add_executable(AppFlemonsSpineLearningCL
    FlemonsSpineModelLearningCL.cpp
    AppFlemonsSpineLearningCL.cpp
    
) 

