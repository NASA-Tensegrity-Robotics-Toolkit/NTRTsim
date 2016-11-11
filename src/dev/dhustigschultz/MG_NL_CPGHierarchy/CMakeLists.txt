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
	       MountainGoatNL
               BaseQuadModelLearning
	       JSONQuadControl)

add_library(JSONHierarchyControlNL
	    JSONHierarchyControlNL.cpp)

add_executable(AppMGHierarchyNL
               AppMGHierarchyNL.cpp
	       JSONHierarchyControlNL.cpp)

target_link_libraries(AppMGHierarchyNL ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatNL)
target_link_libraries(JSONHierarchyControlNL ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatNL KinematicString)
