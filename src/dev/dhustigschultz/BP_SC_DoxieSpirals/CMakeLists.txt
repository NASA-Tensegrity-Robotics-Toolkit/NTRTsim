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
	       BigDoxieSpirals
	       JSONQuadFeedback)

add_executable(AppQuadControlDoxieSpirals
               AppQuadControlDoxieSpirals.cpp)

target_link_libraries(AppQuadControlDoxieSpirals ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
