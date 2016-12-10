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
	       MountainGoatAchillesSpirals2
               BaseQuadModelLearning
	       JSONQuadControl)

add_library(JSONAAS2FeedbackControl
	    JSONAAS2FeedbackControl.cpp)

add_executable(AppAAS2
               AppAAS2.cpp
	       JSONAAS2FeedbackControl.cpp)

target_link_libraries(AppAAS2 ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchillesSpirals2)
target_link_libraries(JSONAAS2FeedbackControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchillesSpirals2 KinematicString)
