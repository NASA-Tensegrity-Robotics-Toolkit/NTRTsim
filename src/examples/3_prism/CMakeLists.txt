link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                OpenGLWindow)

add_executable(AppPrismModel
    PrismModel.cpp
    AppPrismModel.cpp
) 

