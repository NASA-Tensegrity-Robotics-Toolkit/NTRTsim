link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatAchillesLongAAS
	    MountainGoatAchillesLongAAS.cpp)

add_executable(AppMountainGoatAchillesLongAAS
    MountainGoatAchillesLongAAS.cpp
    AppMountainGoatAchillesLongAAS.cpp) 
