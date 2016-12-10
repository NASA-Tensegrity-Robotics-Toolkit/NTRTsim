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
	       MountainGoatNL2Spirals
               BaseQuadModelLearning
	       JSONQuadControl)

add_library(JSON_NL2SpiralsFeedbackControl
	    JSON_NL2SpiralsFeedbackControl.cpp)

add_executable(AppNL2Spirals
               AppNL2Spirals.cpp
	       JSON_NL2SpiralsFeedbackControl.cpp)

target_link_libraries(AppNL2Spirals ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatNL2Spirals)
target_link_libraries(JSON_NL2SpiralsFeedbackControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatNL2Spirals KinematicString)
