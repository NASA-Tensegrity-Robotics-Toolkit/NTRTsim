link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatNL2Spirals
	    MountainGoatNL2Spirals.cpp)

add_executable(AppMountainGoatNL2Spirals
    MountainGoatNL2Spirals.cpp
    AppMountainGoatNL2Spirals.cpp) 
