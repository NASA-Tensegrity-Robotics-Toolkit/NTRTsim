link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(AppPrismModelML
    PrismModelML.cpp
    AppPrismModelML.cpp
) 

