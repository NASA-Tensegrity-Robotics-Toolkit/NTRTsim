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
               controllers)

add_library(JSONMixedLearning
	    JSONMixedLearningControl.cpp)

add_executable(AppMixedLearning
               AppMixedLearning.cpp
	       FlemonsSpineModelMixed.cpp
	       JSONMixedLearningControl.cpp)

target_link_libraries(JSONMixedLearning ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
target_link_libraries(AppMixedLearning ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
