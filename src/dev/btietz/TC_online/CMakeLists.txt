link_libraries(GoalSpine
                obstacles
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
                OpenGLWindow
                boost_program_options)
                
                
add_executable(AppGoalOnline
    SpineOnlineControl.cpp
    AppGoalOnline.cpp
)
