link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatAchilles2
	    MountainGoatAchilles2.cpp)

add_executable(AppMountainGoatAchilles2
    MountainGoatAchilles2.cpp
    AppMountainGoatAchilles2.cpp) 
