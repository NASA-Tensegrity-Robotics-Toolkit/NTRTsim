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
	       BigPuppySymmetricSpiralSegments
               BaseQuadModelLearning
	       JSONQuadControl)

add_library(JSONSegmentsFeedback
	    JSONSegmentsFeedbackControl.cpp)

add_executable(AppQuadControlSegments
               AppQuadControlSegments.cpp
	       JSONSegmentsFeedbackControl.cpp)

target_link_libraries(AppQuadControlSegments ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles BigPuppySpineOnlyStats)
