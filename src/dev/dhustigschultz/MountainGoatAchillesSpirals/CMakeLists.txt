link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatAchillesSpirals
	    MountainGoatAchillesSpirals.cpp)

add_executable(AppMountainGoatAchillesSpirals
    MountainGoatAchillesSpirals.cpp
    AppMountainGoatAchillesSpirals.cpp) 
