link_directories(${LIB_DIR})

link_libraries(Adapters
                Configuration
                NeuroEvolution
                FileHelpers)

add_executable(AppGATests
    AppGATests.cpp
    
) 

add_executable(AppJSONLearningTests
    AppGATests_JSON.cpp
    
)

target_link_libraries(AppJSONLearningTests ${ENV_LIB_DIR}/libjsoncpp.a boost_program_options)
