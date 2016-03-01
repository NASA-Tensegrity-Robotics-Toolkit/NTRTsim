link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(BigPuppySymmetricSpiralMetrics
	    BigPuppySymmetricSpiralMetrics.cpp)

add_executable(AppBigPuppySymmetricSpiralMetrics
    BigPuppySymmetricSpiralMetrics.cpp
    AppBigPuppySymmetricSpiralMetrics.cpp) 
