link_directories(${LIB_DIR})

link_libraries(octahedralComplex
                CPG_feedback
                learningSpines
                sensors
                tgcreator
                core
                util
                terrain
                Adapters
                Configuration
                AnnealEvolution
                FileHelpers
                tgOpenGLSupport)

add_library(JSONControl SHARED
                JSONCPGControl.cpp
                JSONFeedbackControl.cpp
                tgCPGJSONLogger.cpp
                )
                
add_executable(AppJSONTests
	AppJSONTests.cpp
)

add_executable(AppSpineJSON
    JSONCPGControl.cpp
    AppJSONSpine.cpp
)

add_executable(AppTerrainJSON
    JSONCPGControl.cpp
    JSONFeedbackControl.cpp
    tgCPGJSONLogger.cpp
    AppTerrainJSON.cpp
    )

target_link_libraries(AppJSONTests ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
target_link_libraries(AppSpineJSON ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
target_link_libraries(AppTerrainJSON ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles flemonsSpineContact KinematicString)
target_link_libraries(JSONControl ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers boost_program_options obstacles flemonsSpineContact KinematicString)
configure_file("controlVars.json" "controlVars.json" COPYONLY)
configure_file("controlVarsOct.json" "controlVarsOct.json" COPYONLY)

