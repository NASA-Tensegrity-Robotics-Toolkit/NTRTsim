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
               controllers
	       JSONMixedLearning)

add_executable(AppRibbed_MixedLearning
               AppMixedLearning.cpp
	       RibModelMixedContact.cpp)

target_link_libraries(AppRibbed_MixedLearning ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
