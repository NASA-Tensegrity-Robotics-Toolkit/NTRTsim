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

add_executable(App12Bar
    LengthControllerSequential.cpp
    LengthControllerWithReturn.cpp
    LengthControllerMultipleSequential.cpp
    LengthController12BarCube.cpp
    App12Bar.cpp
) 


