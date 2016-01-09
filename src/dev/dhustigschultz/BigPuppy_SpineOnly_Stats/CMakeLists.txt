link_libraries(tgcreator 
	       core
	       learningSpines)

add_library(BigPuppySpineOnlyStats
	    BigPuppySpineOnlyStats.cpp)

add_library(BaseQuadModelLearning
	    BaseQuadModelLearning.cpp)

add_executable(AppBigPuppySpineOnlyStats
    BigPuppySpineOnlyStats.cpp
    AppBigPuppySpineOnlyStats.cpp
    BaseQuadModelLearning.cpp
) 
