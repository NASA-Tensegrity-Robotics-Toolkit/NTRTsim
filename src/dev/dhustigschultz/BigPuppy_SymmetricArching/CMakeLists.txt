link_libraries(tgcreator 
	       core
	       learningSpines
               BaseQuadModelLearning)

add_library(BigPuppySymmetricArching
	    BigPuppySymmetricArching.cpp)

add_executable(AppBigPuppySymmetricArching
    BigPuppySymmetricArching.cpp
    AppBigPuppySymmetricArching.cpp
) 
