link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatSM
	    MountainGoatSM.cpp)

add_executable(AppMountainGoatSM
    MountainGoatSM.cpp
    AppMountainGoatSM.cpp) 
