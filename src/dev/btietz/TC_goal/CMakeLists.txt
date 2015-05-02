link_libraries(obstacles
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

add_library(GoalSpine SHARED
                BaseSpineModelGoal.cpp
                FlemonsSpineModelGoal.cpp
                SpineGoalControl.cpp
                JSONGoalControl.cpp
                )
                
                
add_executable(AppGoalTerrain
    BaseSpineModelGoal.cpp
    FlemonsSpineModelGoal.cpp
    SpineGoalControl.cpp
    JSONGoalControl.cpp
    AppGoalTerrain.cpp
)

target_link_libraries(GoalSpine ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options JSONControl)
target_link_libraries(AppGoalTerrain ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options JSONControl)
