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
	       MountainGoatAchillesLong
               BaseQuadModelLearning
	       JSONQuadControl)

add_library(JSONAOLongControl
	    JSONAOLongControl.cpp)

add_executable(AppAOLong
               AppAOLong.cpp
	       JSONAOLongControl.cpp)

target_link_libraries(AppAOLong ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchillesLong)
target_link_libraries(JSONAOLongControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchillesLong KinematicString)
