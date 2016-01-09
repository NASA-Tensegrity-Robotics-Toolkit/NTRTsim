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
	       BigDoxie
	       JSONQuadFeedback)

add_executable(AppQuadControlDoxie
               AppQuadControlDoxie.cpp)

target_link_libraries(AppQuadControlDoxie ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
