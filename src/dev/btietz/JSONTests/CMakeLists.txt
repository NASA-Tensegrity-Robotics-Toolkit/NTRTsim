link_directories(${LIB_DIR})

add_executable(AppJSONTests
	AppJSONTests.cpp
)

target_link_libraries(AppJSONTests ${ENV_LIB_DIR}/libjsoncpp.a FileHelpers)
configure_file("controlVars.json" "controlVars.json" COPYONLY)

