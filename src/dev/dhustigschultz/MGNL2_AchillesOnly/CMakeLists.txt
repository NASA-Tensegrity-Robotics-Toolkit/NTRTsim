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
	       MountainGoatNL2
               BaseQuadModelLearning
	       JSONQuadControl)

add_library(JSON_AO_FeedbackControl
	    JSON_AO_FeedbackControl.cpp)

add_executable(AppMGNL2_AO
               AppMGNL2_AO.cpp
	       JSON_AO_FeedbackControl.cpp)

target_link_libraries(AppMGNL2_AO ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatNL2)
target_link_libraries(JSON_AO_FeedbackControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatNL2 KinematicString)
