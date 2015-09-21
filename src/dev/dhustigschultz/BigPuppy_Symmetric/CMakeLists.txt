link_libraries(tgcreator 
	       core
	       learningSpines)

add_library(BigPuppySymmetric
	    BigPuppySymmetric.cpp)

add_executable(AppBigPuppySymmetric
    BigPuppySymmetric.cpp
    AppBigPuppySymmetric.cpp
) 
