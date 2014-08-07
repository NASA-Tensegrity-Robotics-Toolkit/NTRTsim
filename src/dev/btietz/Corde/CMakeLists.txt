link_directories(${LIB_DIR})

link_libraries(btietz
				tgcreator
                util
                sensors
                core
                Corde    
                terrain 
                tgOpenGLSupport)


add_executable(AppCordeTest
    AppCordeTest.cpp
)

add_executable(AppVectorTest
	AppVectorTest.cpp
)
