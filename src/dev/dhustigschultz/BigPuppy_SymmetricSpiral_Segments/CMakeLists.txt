link_libraries(tgcreator 
	       core
               BaseQuadModelLearning)

add_library(BigPuppySymmetricSpiralSegments
	    BigPuppySymmetricSpiralSegments.cpp)

add_executable(AppBigPuppySymmetricSpiralSegments
    BigPuppySymmetricSpiralSegments.cpp
    AppBigPuppySymmetricSpiralSegments.cpp) 
