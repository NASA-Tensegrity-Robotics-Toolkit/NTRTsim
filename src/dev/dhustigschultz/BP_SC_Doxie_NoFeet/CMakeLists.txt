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
               sensors
               controllers
	       BigDoxieNoFeet
	       JSONQuadFeedback)

add_executable(AppQuadControlDoxieNF
               AppQuadControlDoxieNF.cpp)

target_link_libraries(AppQuadControlDoxieNF ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles JSONControl)
