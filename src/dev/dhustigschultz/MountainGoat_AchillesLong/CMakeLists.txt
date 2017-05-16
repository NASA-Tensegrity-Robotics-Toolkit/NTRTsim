link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatAchillesLong
	    MountainGoatAchillesLong.cpp)

add_executable(AppMountainGoatAchillesLong
    MountainGoatAchillesLong.cpp
    AppMountainGoatAchillesLong.cpp) 
