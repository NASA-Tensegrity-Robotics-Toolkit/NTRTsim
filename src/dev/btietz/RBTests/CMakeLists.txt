link_directories(${LIB_DIR})

link_libraries(btietz
				tgcreator
                util
                sensors
                core    
                terrain 
                OpenGLWindow)


add_executable(AppContactModelTest
    AppContactModelTest.cpp
) 
