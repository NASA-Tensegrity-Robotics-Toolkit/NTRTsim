#sublink_directories(${LIB_DIR})

link_directories(${LIB_DIR})

message(STATUS "LIB_DIR: " ${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport
    		yaml-cpp
    		TensegrityModel)

# add_library(TensegrityModel
#    TensegrityModel.cpp)

add_executable(App12Bar
#    TensegrityModel.cpp
    App12Bar.cpp)
#    RPLengthController.h
#    RPLengthController_rand.cpp)


