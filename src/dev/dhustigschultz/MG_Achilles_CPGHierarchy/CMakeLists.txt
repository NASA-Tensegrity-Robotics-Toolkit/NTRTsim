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

add_library(JSONAchillesHierarchyControl
	    JSONAchillesHierarchyControl.cpp)

add_executable(AppAchillesHierarchy
               AppAchillesHierarchy.cpp
	       JSONAchillesHierarchyControl.cpp)

target_link_libraries(AppAchillesHierarchy ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchilles)
target_link_libraries(JSONAchillesHierarchyControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchilles KinematicString)
