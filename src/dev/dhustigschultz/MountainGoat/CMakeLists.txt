link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoat
	    MountainGoat.cpp)

add_executable(AppMountainGoat
    MountainGoat.cpp
    AppMountainGoat.cpp) 
