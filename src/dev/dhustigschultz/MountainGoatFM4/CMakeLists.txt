link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatFM4
	    MountainGoatFM4.cpp)

add_executable(AppMountainGoatFM4
    MountainGoatFM4.cpp
    AppMountainGoatFM4.cpp) 
