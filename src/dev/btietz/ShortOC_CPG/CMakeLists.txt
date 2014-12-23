link_directories(${LIB_DIR})

link_libraries(octahedralComplex
                CPG_feedback
                learningSpines
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

add_executable(AppOctaCL_CPG
    AppOctaCL_CPG.cpp
    
) 

