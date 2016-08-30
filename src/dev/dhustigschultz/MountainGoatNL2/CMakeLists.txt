link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatNL2
	    MountainGoatNL2.cpp)

add_executable(AppMountainGoatNL2
    MountainGoatNL2.cpp
    AppMountainGoatNL2.cpp) 
