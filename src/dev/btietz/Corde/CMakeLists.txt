link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)


add_executable(AppCordeTest
    CordeModel.cpp
    AppCordeTest.cpp
) 

add_executable(AppLineInsertionCheck
	AppLineInsertionCheck.cpp
)
