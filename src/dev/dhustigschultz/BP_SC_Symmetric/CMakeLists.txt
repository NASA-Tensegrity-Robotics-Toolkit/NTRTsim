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
	       BigPuppySymmetric)

add_library(JSONQuadFeedback
	    JSONQuadFeedbackControl.cpp)

add_executable(AppQuadControl
               AppQuadControl.cpp
	       JSONQuadFeedbackControl.cpp)

target_link_libraries(JSONQuadFeedback ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
target_link_libraries(AppQuadControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
