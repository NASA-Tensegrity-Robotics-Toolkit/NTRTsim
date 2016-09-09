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
	       MountainGoatFM0
	       BaseQuadModelLearning
               JSONQuadControl)

add_library(JSONMGFeedbackFM0
	    JSONMGFeedbackControlFM0.cpp)

add_executable(AppMGControlFM0
               AppMGControlFM0.cpp
	       JSONMGFeedbackControlFM0.cpp)

target_link_libraries(JSONMGFeedbackFM0 ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONQuadControl)
target_link_libraries(AppMGControlFM0 ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONQuadControl)
