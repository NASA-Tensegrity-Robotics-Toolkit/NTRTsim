link_libraries(tgcreator 
               core
	       MGSingleLeg
               BaseQuadModelLearning)

add_executable(AppMGSingleLegHT
    MGSingleLegController.cpp
    AppMGSingleLegHT.cpp
) 
