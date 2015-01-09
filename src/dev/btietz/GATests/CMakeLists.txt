link_directories(${LIB_DIR})

link_libraries(Adapters
                Configuration
                NeuroEvolution
                FileHelpers)

add_executable(AppGATests
    AppGATests.cpp
    
) 
