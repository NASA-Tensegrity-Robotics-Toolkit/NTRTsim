#sublink_directories(${LIB_DIR})

link_directories(${LIB_DIR})

message(STATUS "LIB_DIR: " ${LIB_DIR})

link_libraries(tgcreator
                controllers
                core
	        sensors
                util   
                terrain 
                tgOpenGLSupport
        	yaml-cpp
    		TensegrityModel
#		learning
		learningSpines
		Adapters
		Configuration	
		AnnealEvolution)



add_executable(App12BarGround
	T12ModelGround.cpp
	App12BarGround.cpp
	T12ControllerGround.cpp)
