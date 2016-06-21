link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(AppThruster_Active
    PrismModel.cpp
    AppPrismModel.cpp
    RPThruster.cpp
) 

