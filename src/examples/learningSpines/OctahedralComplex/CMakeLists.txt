link_directories(${LIB_DIR})

link_libraries(learningSpines
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

add_executable(AppFlemonsSpineLearningCL
    FlemonsSpineModelLearningCL.cpp
    AppFlemonsSpineLearningCL.cpp
    
) 

add_custom_command(TARGET AppFlemonsSpineLearningCL POST_BUILD
					COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/logs)
