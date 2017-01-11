link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatFM5
	    MountainGoatFM5.cpp)

add_executable(AppMountainGoatFM5
    MountainGoatFM5.cpp
    AppMountainGoatFM5.cpp) 
