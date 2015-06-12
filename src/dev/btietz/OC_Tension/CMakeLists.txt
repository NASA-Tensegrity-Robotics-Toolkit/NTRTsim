link_libraries(JSONGoalTension
                OctahedralComplex
                GoalSpine
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

add_executable(AppOC_Tension
    OctahedralTensionControl.cpp
    AppOCTension.cpp
)

target_link_libraries(AppOC_Tension ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
