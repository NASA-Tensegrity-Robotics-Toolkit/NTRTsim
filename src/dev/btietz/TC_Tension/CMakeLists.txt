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

add_library(JSONGoalTension
    JSONGoalTension.cpp)
                
add_executable(AppTC_Tension
    JSONGoalTension.cpp
    AppGoalTension.cpp
)

target_link_libraries(JSONGoalTension ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
target_link_libraries(AppTC_Tension ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
