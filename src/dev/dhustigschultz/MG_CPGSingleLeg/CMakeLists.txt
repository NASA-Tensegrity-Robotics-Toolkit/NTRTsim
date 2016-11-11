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
	       MGSingleLeg
               BaseQuadModelLearning
	       JSONQuadControl)

add_library(JSONMGSingleLegControl
	    JSONMGSingleLegControl.cpp)

add_executable(AppMGSingleLegControl
               AppMGSingleLegControl.cpp
	       JSONMGSingleLegControl.cpp)

target_link_libraries(AppMGSingleLegControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MGSingleLeg)
target_link_libraries(JSONMGSingleLegControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MGSingleLeg KinematicString)
