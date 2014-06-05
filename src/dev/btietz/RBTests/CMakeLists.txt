link_directories(${LIB_DIR})

link_libraries(btietz
				tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)


add_executable(AppContactModelTest
    AppContactModelTest.cpp
) 
