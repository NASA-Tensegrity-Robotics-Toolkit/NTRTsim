link_directories(${LIB_DIR})

link_libraries( Adapters
                tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(AppEscape
    EscapeModel.cpp
    EscapeController.cpp
    AppEscape.cpp
) 

