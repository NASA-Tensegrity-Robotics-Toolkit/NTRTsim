link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                Adapters
                Configuration
                AnnealEvolution
                FileHelpers
                core    
                terrain 
                tgOpenGLSupport)

add_executable(AppEscape
    EscapeModel.cpp
    EscapeController.cpp
    AppEscape.cpp
) 

