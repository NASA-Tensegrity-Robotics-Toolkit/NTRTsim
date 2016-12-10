link_libraries(boost_program_options
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
	       MountainGoatAchillesSpirals
               BaseQuadModelLearning
	       JSONQuadControl)

add_library(JSONAASFeedbackControl
	    JSONAASFeedbackControl.cpp)

add_executable(AppAAS
               AppAAS.cpp
	       JSONAASFeedbackControl.cpp)

target_link_libraries(AppAAS ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchillesSpirals)
target_link_libraries(JSONAASFeedbackControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchillesSpirals KinematicString)
