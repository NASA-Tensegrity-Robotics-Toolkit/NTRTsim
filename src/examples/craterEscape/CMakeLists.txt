link_directories(${LIB_DIR})

link_libraries(obstacles 
                tgcreator
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

ntrt_add_runme(AppEscape DESCRIPTION "Run the crater escape demo")
