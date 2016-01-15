link_libraries(GoalSpine
                obstacles
                KinematicString
               flemonsSpineContact
               CPG_feedback
                KinematicString
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

add_library(OctahedralComplex
    OctahedralComplex.cpp
    )
                
add_executable(AppMultiTerrainOC
    OctahedralComplex.cpp
    OctahedralGoalControl.cpp
    AppMultiTerrain_OC.cpp
)

target_link_libraries(OctahedralComplex ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
target_link_libraries(AppMultiTerrainOC ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
