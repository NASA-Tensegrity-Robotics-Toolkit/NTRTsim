link_libraries(obstacles
               flemonsSpineContact
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

add_executable(AppMultiTerrain
    AppMultiTerrain.cpp
)
