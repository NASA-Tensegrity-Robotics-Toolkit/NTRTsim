link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatFM7
	    MountainGoatFM7.cpp)

add_executable(AppMountainGoatFM7
    MountainGoatFM7.cpp
    AppMountainGoatFM7.cpp) 
