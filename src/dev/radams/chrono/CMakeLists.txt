link_libraries(boost_system boost_chrono)

link_directories(${LIB_DIR})

include_directories(${INC_DIR})

add_executable(AppChronoTest
    chronoTest.cpp
) 
