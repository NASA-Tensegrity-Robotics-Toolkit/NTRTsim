link_libraries(${ENV_LIB_DIR})
add_executable(TestJSON testjson.cpp)
target_link_libraries(TestJSON ${ENV_LIB_DIR}/libjsoncpp.a)
