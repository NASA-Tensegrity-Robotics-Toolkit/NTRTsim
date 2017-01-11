link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatFM6
	    MountainGoatFM6.cpp)

add_executable(AppMountainGoatFM6
    MountainGoatFM6.cpp
    AppMountainGoatFM6.cpp) 
