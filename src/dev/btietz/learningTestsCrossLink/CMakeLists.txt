link_directories(${LIB_DIR})

link_libraries(btietz
                sensors
                tgcreator
                core
                util
                terrain
                Adapters
                Configuration
                AnnealEvolution
                tgOpenGLSupport)

file(COPY nodeConfig.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY edgeConfig.ini DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
#ensure logs exists
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/logs)

add_executable(AppFlemonsSpineLearningCL
    FlemonsSpineModelLearning.cpp
    AppFlemonsSpineLearning.cpp
    
) 
