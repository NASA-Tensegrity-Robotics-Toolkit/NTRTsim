link_directories(${LIB_DIR})

link_libraries(learningSpines
                sensors
                tgcreator
                MuscleNP
                core
                util
                terrain
                Adapters
                Configuration
                AnnealEvolution
                tgOpenGLSupport)

add_executable(AppFlemonsSpineLearningCL
    FlemonsSpineModelLearningCL.cpp
    AppFlemonsSpineLearningCL.cpp
    
) 

