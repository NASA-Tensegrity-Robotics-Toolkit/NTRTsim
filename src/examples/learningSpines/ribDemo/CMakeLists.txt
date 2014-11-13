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

add_executable(AppRibModelLearning
    RibModel.cpp
    AppRibModelLearning.cpp
    
) 

