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
	       MountainGoatAchillesLongAAS
               BaseQuadModelLearning
	       JSONQuadControl)

add_library(JSONAASLongControl
	    JSONAASLongControl.cpp)

add_executable(AppAASLong
               AppAASLong.cpp
	       JSONAASLongControl.cpp)

target_link_libraries(AppAASLong ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchillesLongAAS)
target_link_libraries(JSONAASLongControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchillesLongAAS KinematicString)
