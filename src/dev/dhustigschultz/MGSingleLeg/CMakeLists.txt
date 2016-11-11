link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(MGSingleLeg
	    MGSingleLeg.cpp)

add_executable(AppMGSingleLeg
    MGSingleLeg.cpp
    AppMGSingleLeg.cpp) 
