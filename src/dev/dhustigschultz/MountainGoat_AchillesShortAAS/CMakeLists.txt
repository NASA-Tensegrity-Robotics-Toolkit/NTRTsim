link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatAchillesShortAAS
	    MountainGoatAchillesShortAAS.cpp)

add_executable(AppMountainGoatAchillesShortAAS
    MountainGoatAchillesShortAAS.cpp
    AppMountainGoatAchillesShortAAS.cpp) 
