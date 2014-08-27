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
