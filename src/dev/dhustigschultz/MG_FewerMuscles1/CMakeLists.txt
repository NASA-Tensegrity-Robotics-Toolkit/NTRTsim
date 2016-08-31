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
	       MountainGoat
	       BaseQuadModelLearning
               JSONQuadControl)

add_library(JSONMGFeedbackFM1
	    JSONMGFeedbackControlFM1.cpp)

add_executable(AppMGControlFM1
               AppMGControlFM1.cpp
	       JSONMGFeedbackControlFM1.cpp)

target_link_libraries(JSONMGFeedbackFM1 ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONQuadControl)
target_link_libraries(AppMGControlFM1 ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONQuadControl)
