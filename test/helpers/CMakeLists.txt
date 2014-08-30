configure_file(test_file.txt test_file.txt COPYONLY)
add_executable(FileHelpers_test
    FileHelpers_test.cpp)

target_link_libraries(FileHelpers_test ${ENV_LIB_DIR}/libgtest.a pthread)
