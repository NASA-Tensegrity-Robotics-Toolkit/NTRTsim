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
    		    TensegrityModel)

#add_executable(App12Bar
 #   LengthControllerSequential.cpp
  #  LengthControllerWithReturn.cpp
   # LengthControllerMultipleSequential.cpp
#    LengthController12BarCube.cpp
   # App12Bar.cpp
#)

add_executable(App12BarTest1
    LengthControllerSequential.cpp
    LengthControllerWithReturn.cpp
    LengthControllerMultipleSequential.cpp
 #   LengthController12BarCube.cpp
    App12BarTest1.cpp
) 
 
add_executable(App12BarTest2
    LengthControllerSequential.cpp
    LengthControllerWithReturn.cpp
    LengthController12BarOctahedron.cpp    
    LengthControllerMultipleSequential.cpp
    App12BarTest2.cpp
)

#add_executable(App12BarOctahedron
 #   LengthController12BarOctahedron.cpp
  #  App12BarOctahedron.cpp
#)
