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
                FlemonsSpineModelGoal.cpp
                SpineGoalControl.cpp
                )
                
                
add_executable(AppGoalTerrain
    FlemonsSpineModelGoal.cpp
    SpineGoalControl.cpp
    AppGoalTerrain.cpp
)
