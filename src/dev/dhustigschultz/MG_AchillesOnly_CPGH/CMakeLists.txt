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
	       MountainGoatAchilles
               BaseQuadModelLearning
	       JSONQuadControl)

add_library(JSONAOHierarchyControl
	    JSONAOHierarchyControl.cpp)

add_executable(AppAOHierarchy
               AppAOHierarchy.cpp
	       JSONAOHierarchyControl.cpp)

target_link_libraries(AppAOHierarchy ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchilles)
target_link_libraries(JSONAOHierarchyControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchilles KinematicString)
