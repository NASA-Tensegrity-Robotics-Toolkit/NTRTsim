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
	       BigPuppySymmetricSpiral2
	       JSONQuadFeedback)

add_executable(AppQuadControlSpiral2
               AppQuadControlSpiral2.cpp)

target_link_libraries(AppQuadControlSpiral2 ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
