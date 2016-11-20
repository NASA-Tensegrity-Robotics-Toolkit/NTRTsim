link_directories(${LIB_DIR})

link_libraries(controllers
				tgcreator
                util
                sensors
                learningSpines
                controllers
                core    
                terrain 
                tgOpenGLSupport)


add_executable(AppPIDTest
				pidTestRig.cpp
				AppPIDTest.cpp
				)
