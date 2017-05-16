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
	       MountainGoatAchillesShort
               BaseQuadModelLearning
	       JSONQuadControl)

add_library(JSONAOShortControl
	    JSONAOShortControl.cpp)

add_executable(AppAOShort
               AppAOShort.cpp
	       JSONAOShortControl.cpp)

target_link_libraries(AppAOShort ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchillesShort)
target_link_libraries(JSONAOShortControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles MountainGoatAchillesShort KinematicString)
