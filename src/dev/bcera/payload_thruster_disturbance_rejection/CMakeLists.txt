link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(App_Payload_Thruster_DR
    PrismModel.cpp
    AppPrismModel.cpp
    RPThruster.cpp
) 

