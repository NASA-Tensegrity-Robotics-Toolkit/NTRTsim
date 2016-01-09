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
	       BigPuppySpineOnlyStats
               BaseQuadModelLearning
	       JSONControl)


add_library(JSONQuadControl SHARED
                JSONQuadCPGControl.cpp
                JSONStatsFeedbackControl.cpp)

add_executable(AppSpineControlStats
               AppSpineControlStats.cpp
	       JSONStatsFeedbackControl.cpp
	       JSONQuadCPGControl.cpp)

target_link_libraries(AppSpineControlStats ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles BigPuppySpineOnlyStats)
target_link_libraries(JSONQuadControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles BigPuppySpineOnlyStats KinematicString)
