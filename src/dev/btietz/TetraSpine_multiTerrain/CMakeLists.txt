link_libraries(obstacles
                learningSpines
                sensors
                controllers
                tgcreator             
                core
                util
                terrain
                Adapters
                Configuration
                AnnealEvolution
                tgOpenGLSupport
                boost_program_options)

add_executable(AppMultiTerrain_Tetra
    TetraSpineKinematic.cpp
    AppMultiTerrain_Tetra.cpp
)
