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
	       MountainGoatAchilles2
               BaseQuadModelLearning
	       JSONQuadControl)

add_library(JSONAO2FeedbackControl
	    JSONAO2FeedbackControl.cpp)

add_executable(AppAO2
               AppAO2.cpp
	       JSONAO2FeedbackControl.cpp)

target_link_libraries(AppAO2 ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchilles2)
target_link_libraries(JSONAO2FeedbackControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchilles2 KinematicString)
