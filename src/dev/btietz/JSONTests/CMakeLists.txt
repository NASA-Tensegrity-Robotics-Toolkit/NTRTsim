link_directories(${LIB_DIR})

link_libraries(octahedralComplex
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

add_executable(AppJSONTests
	AppJSONTests.cpp
)

add_executable(AppSpineJSON
    JSONCPGControl.cpp
    AppJSONSpine.cpp
)

target_link_libraries(AppJSONTests ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
target_link_libraries(AppSpineJSON ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
configure_file("controlVars.json" "controlVars.json" COPYONLY)
configure_file("controlVarsOct.json" "controlVarsOct.json" COPYONLY)

