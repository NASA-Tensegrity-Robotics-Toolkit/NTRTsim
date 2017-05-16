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
	       MountainGoatAchillesShortAAS
               BaseQuadModelLearning
	       JSONQuadControl)

add_library(JSONAASShortControl
	    JSONAASShortControl.cpp)

add_executable(AppAASShort
               AppAASShort.cpp
	       JSONAASShortControl.cpp)

target_link_libraries(AppAASShort ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchillesShortAAS)
target_link_libraries(JSONAASShortControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchillesShortAAS KinematicString)
