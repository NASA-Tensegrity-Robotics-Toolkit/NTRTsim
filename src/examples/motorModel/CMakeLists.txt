link_directories(${LIB_DIR})

link_libraries(controllers
				tgcreator
                util
                sensors
                controllers
                core    
                terrain 
                tgOpenGLSupport)

add_library(TimestepTest SHARED
			tsTestRig.cpp)

add_executable(AppTSTest
    tsTestRig.cpp
    AppTimestepTest.cpp
) 
