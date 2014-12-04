link_directories(${LIB_DIR})

link_libraries(controllers
				tgcreator
                util
                sensors
                KinematicString
                core    
                terrain 
                tgOpenGLSupport)

add_executable(AppTSTest
    tsTestRig.cpp
    AppTimestepTest.cpp
) 

