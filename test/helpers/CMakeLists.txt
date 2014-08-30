configure_file(test_file.txt test_file.txt COPYONLY)
add_executable(FileHelpers_test
    FileHelpers_test.cpp)
