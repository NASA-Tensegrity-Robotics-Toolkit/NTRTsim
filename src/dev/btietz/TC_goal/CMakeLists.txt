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
                )
                
                
add_executable(AppGoalTerrain
    BaseSpineModelGoal.cpp
    FlemonsSpineModelGoal.cpp
    SpineGoalControl.cpp
    AppGoalTerrain.cpp
)
