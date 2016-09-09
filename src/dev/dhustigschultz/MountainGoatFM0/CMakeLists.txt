link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatFM0
	    MountainGoatFM0.cpp)

add_executable(AppMountainGoatFM0
    MountainGoatFM0.cpp
    AppMountainGoatFM0.cpp) 
