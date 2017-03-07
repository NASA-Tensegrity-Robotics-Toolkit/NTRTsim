link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatAchilles
	    MountainGoatAchilles.cpp)

add_executable(AppMountainGoatAchilles
    MountainGoatAchilles.cpp
    AppMountainGoatAchilles.cpp) 
