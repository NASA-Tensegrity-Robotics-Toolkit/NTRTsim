link_libraries(JSONQuadControl
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
	       BaseQuadModelLearning
	       BigPuppySymmetricArching)

add_library(JSONNonlinearFeedback
	    JSONNonlinearFeedbackControl.cpp)

add_executable(AppQuadControlArching
               AppQuadControlArching.cpp
	       JSONNonlinearFeedbackControl.cpp)

target_link_libraries(JSONNonlinearFeedback ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONQuadControl)
target_link_libraries(AppQuadControlArching ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONQuadControl)
