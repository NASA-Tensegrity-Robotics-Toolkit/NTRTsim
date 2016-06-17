link_directories(${LIB_DIR})

message(STATUS "LIB_DIR: " ${LIB_DIR})

link_libraries(core
               tgcreator
               controllers
               util
               sensors
               terrain
               tgOpenGLSupport
               yaml-cpp
	       Adapters
	       TensegrityModel)

# add_library(TensegrityModel
#     TensegrityModel.cpp)

add_executable(App6Bar
#    TensegrityModel.cpp
    RPLengthController.h
    RPLengthController_rand.cpp
    App6Bar.cpp)


