#sublink_directories(${LIB_DIR})

#find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs geometry_msgs)

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


add_executable(App12BarCpp
	T12Model.cpp
	App12BarCpp.cpp
	T12Controller.cpp)
