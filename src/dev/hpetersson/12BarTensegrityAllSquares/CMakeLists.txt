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


#add_executable(App12Bar
 #   LengthControllerSequential.cpp
  #  LengthControllerWithReturn.cpp
   # LengthControllerMultipleSequential.cpp
#    Leng
#thController12BarCube.cpp
   # App12Bar.cpp
#)

#add_executable(App12BarTest1
#    LengthController12BarOctahedron.cpp
#    App12BarTest1.cpp
#) 
 
#add_executable(App12BarTest2
#    LengthController12BarOctahedron.cpp    
#    App12BarTest2.cpp
#)


#link_directories(${LIB_DIR})

#link_libraries(learningSpines
#                sensors
 #               tgcreator
  #              core
   #             util
    #            terrain
     #           Adapters
      #          Configuration
       #         AnnealEvolution
        #        tgOpenGLSupport)

#add_library (octahedralComplex SHARED
                               # FlemonsSpineModelLearningCL.cpp)

#add_library (octahedralComplex SHARED
#			ModelLearning12Bar.cpp)


#add_executable(AppLearning12Bar
#    LearningController12Bar.cpp
#    AppLearning12Bar.cpp) 

add_executable(App12BarCpp
	T12Model.cpp
	App12BarCpp.cpp
	T12Controller.cpp)
