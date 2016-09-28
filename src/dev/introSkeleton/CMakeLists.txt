link_directories(${LIB_DIR})

message(STATUS "LIB_DIR: " ${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(App3Bar
    threeBarModel.cpp
    App3Bar.cpp
) 

