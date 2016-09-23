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
	       MountainGoat
               BaseQuadModelLearning
	       JSONQuadControl)

add_library(JSONHierarchyFeedbackControl
	    JSONHierarchyFeedbackControl.cpp)

add_executable(AppQuadControlHierarchy
               AppQuadControlHierarchy.cpp
	       JSONHierarchyFeedbackControl.cpp)

target_link_libraries(AppQuadControlHierarchy ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoat)
target_link_libraries(JSONHierarchyFeedbackControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoat KinematicString)
