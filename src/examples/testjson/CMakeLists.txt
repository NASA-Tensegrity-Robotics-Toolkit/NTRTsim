add_executable(TestJSON testjson.cpp)
target_link_libraries(TestJSON ${ENV_LIB_DIR}/libjsoncpp.a
    FileHelpers)
configure_file(data.txt data.txt COPYONLY)
