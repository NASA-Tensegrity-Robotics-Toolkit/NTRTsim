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
	       BigPuppySymmetricSpiralMetrics
               BaseQuadModelLearning
	       JSONQuadControl)

add_library(JSONMetricsFeedbackControl
	    JSONMetricsFeedbackControl.cpp)

add_executable(AppQuadControlMetrics
               AppQuadControlMetrics.cpp
	       JSONMetricsFeedbackControl.cpp)

target_link_libraries(AppQuadControlMetrics ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles BigPuppySymmetricSpiralMetrics)
target_link_libraries(JSONMetricsFeedbackControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles BigPuppySymmetricSpiralMetrics KinematicString)
