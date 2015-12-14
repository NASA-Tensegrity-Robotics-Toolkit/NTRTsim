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
	       BigPuppySymmetricSpiral1
	       JSONQuadFeedback)

add_executable(AppQuadControlSpiral
               AppQuadControlSpiral.cpp)

target_link_libraries(AppQuadControlSpiral ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
