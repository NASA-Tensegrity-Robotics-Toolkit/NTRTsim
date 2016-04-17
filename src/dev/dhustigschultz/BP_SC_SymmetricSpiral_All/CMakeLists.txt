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
	       BigPuppySymmetricSpiralSegments
	       tgCPGGeneralActuatorControl
	       tgCPGGeneralCableControl
	       JSONQuadCPGGeneralControl)

add_library(tgCPGGeneralActuatorControl
	    tgCPGGeneralActuatorControl.cpp)

add_library(tgCPGGeneralCableControl
	    tgCPGGeneralCableControl.cpp)

add_library(JSONQuadCPGGeneralControl
	    JSONQuadCPGGeneralControl.cpp)

add_executable(AppQuadControlGeneral
               AppQuadControlGeneral.cpp
	       JSONGeneralFeedbackControl.cpp
	       JSONQuadCPGGeneralControl.cpp
	       tgCPGGeneralCableControl.cpp
	       tgCPGGeneralActuatorControl.cpp)

target_link_libraries(AppQuadControlGeneral ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles)
