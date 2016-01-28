link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(AppHung2ControlTFModel
    Hung2ControlTFModel.cpp
    Hung2ControlTFController.cpp
    AppHung2ControlTFModel.cpp
) 

