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

                
add_executable(AppTCNNW_Tension
    JSONGoalTensionNNW.cpp
    AppGoalTensionNNW.cpp
)

target_link_libraries(AppTCNNW_Tension ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
