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
	       MountainGoatFM4
	       BaseQuadModelLearning
               JSONQuadControl)

add_library(JSONMGFeedbackFM4
	    JSONMGFeedbackControlFM4.cpp)

add_executable(AppMGControlFM4
               AppMGControlFM4.cpp
	       JSONMGFeedbackControlFM4.cpp)

target_link_libraries(JSONMGFeedbackFM4 ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONQuadControl)
target_link_libraries(AppMGControlFM4 ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONQuadControl)
