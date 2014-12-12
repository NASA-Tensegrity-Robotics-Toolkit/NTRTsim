link_directories(${LIB_DIR})

link_libraries(octahedralComplex
				tetraSpineLearningSine
				learningSpines
                sensors
                tgcreator
                controllers
                core
                util
                terrain
                Adapters
                Configuration
                AnnealEvolution
                tgOpenGLSupport)

add_executable(AppOctaCL_sine
    AppOctaCL_sine.cpp
    OctaCLSine.cpp
) 

