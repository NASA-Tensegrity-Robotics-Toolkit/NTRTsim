link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(AppTFModel
    TFModel.cpp
    AppTFModel.cpp
   FlexController.cpp
) 

