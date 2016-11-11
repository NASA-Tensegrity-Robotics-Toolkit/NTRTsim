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
	       MountainGoatSM
               BaseQuadModelLearning
	       JSONQuadControl)

add_library(JSONHierarchyFeedbackControlSM
	    JSONHierarchyFeedbackControlSM.cpp)

add_executable(AppQuadControlHierarchySM
               AppQuadControlHierarchySM.cpp
	       JSONHierarchyFeedbackControlSM.cpp)

target_link_libraries(AppQuadControlHierarchySM ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatSM)
target_link_libraries(JSONHierarchyFeedbackControlSM ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatSM KinematicString)
