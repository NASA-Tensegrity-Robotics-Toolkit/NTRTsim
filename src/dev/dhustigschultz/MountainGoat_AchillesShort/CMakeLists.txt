link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatAchillesShort
	    MountainGoatAchillesShort.cpp)

add_executable(AppMountainGoatAchillesShort
    MountainGoatAchillesShort.cpp
    AppMountainGoatAchillesShort.cpp) 
