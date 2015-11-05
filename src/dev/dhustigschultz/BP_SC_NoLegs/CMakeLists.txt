link_libraries(learningSpines
               boost_program_options
               tgcreator             
               core
               util
               terrain
               Adapters
               Configuration
               AnnealEvolution
               tgOpenGLSupport
               obstacles
               KinematicString
               CPG_feedback
               sensors
               controllers
	       BigPuppySpineOnly
	       JSONQuadFeedback)

add_executable(AppSpineControl
               AppSpineControl.cpp)

target_link_libraries(AppSpineControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
