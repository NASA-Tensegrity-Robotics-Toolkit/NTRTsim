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
	       BaseQuadModelLearning
	       MountainGoat)

add_library(tgCPGGMGActuatorControl
	    tgCPGMGActuatorControl.cpp)

add_library(tgCPGMGCableControl
	    tgCPGMGCableControl.cpp)

add_library(JSONMGCPGGeneralControl
	    JSONMGCPGGeneralControl.cpp)

add_library(JSONMGFeedbackControl
	    JSONMGFeedbackControl.cpp)

add_executable(AppMGControl
               AppMGControl.cpp
	       JSONMGFeedbackControl.cpp
	       JSONMGCPGGeneralControl.cpp
	       tgCPGMGCableControl.cpp
	       tgCPGMGActuatorControl.cpp)

target_link_libraries(AppMGControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles)
