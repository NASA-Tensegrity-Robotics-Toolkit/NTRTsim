CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
project(neuralNetwork)

ADD_LIBRARY( ${PROJECT_NAME}
dataReader.cpp
neuralNetwork.cpp
neuralNetworkTrainer.cpp
)

target_link_libraries(${PROJECT_NAME})

# Install instructions for this target
INSTALL( TARGETS neuralNetwork
         RUNTIME DESTINATION bin
         LIBRARY DESTINATION lib
         ARCHIVE DESTINATION lib
)
INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
DESTINATION ${CMAKE_INSTALL_PREFIX}/include/neuralNet FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE PATTERN "CMakeFiles" EXCLUDE)
