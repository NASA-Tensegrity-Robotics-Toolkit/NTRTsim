link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(App_TT3_Payload_Thruster
    PrismModel.cpp
    AppPrismModel.cpp
    RPThruster.cpp
) 

