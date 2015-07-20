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

add_executable(AppTSGoal
    TetraSpineGoal.cpp
    AppTSGoal.cpp
)

target_link_libraries(AppTSGoal ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
