link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatAchillesSpirals2
	    MountainGoatAchillesSpirals2.cpp)

add_executable(AppMountainGoatAchillesSpirals2
    MountainGoatAchillesSpirals2.cpp
    AppMountainGoatAchillesSpirals2.cpp) 
