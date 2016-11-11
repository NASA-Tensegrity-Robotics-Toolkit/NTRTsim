link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MountainGoatAnchored
            MountainGoatAnchored.cpp)

add_executable(AppMountainGoatAnchored
    MountainGoatAnchored.cpp
    AppMountainGoatAnchored.cpp) 
