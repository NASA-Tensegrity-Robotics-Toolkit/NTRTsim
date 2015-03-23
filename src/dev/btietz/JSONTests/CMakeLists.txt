link_directories(${LIB_DIR})

link_libraries(learningSpines
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

add_executable(AppJSONTests
    JSONCPGControl.cpp
	AppJSONTests.cpp
)

target_link_libraries(AppJSONTests ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
configure_file("controlVars.json" "controlVars.json" COPYONLY)

