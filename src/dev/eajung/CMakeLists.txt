link_directories(${LIB_DIR})

link_libraries(tgcreator
                util
                sensors
                core    
                terrain 
                tgOpenGLSupport)

add_executable(FlemonsArm
	FlemonsArm.cpp
	AppFlemonsArm.cpp
) 

