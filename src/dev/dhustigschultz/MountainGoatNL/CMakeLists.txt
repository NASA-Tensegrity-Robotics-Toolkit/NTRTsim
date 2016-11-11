link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatNL
	    MountainGoatNL.cpp)

add_executable(AppMountainGoatNL
    MountainGoatNL.cpp
    AppMountainGoatNL.cpp) 
