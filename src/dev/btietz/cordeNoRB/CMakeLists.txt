link_directories(${LIB_DIR})

link_libraries(btietz
				tgcreator
                util
                sensors
                core
                Corde    
                terrain 
                tgOpenGLSupport)


add_executable(AppCordeSolo
    soloCorde.cpp
    AppCordeSolo.cpp
)
