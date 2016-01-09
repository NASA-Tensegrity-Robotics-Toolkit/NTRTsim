link_libraries(tgcreator 
	       core
	       learningSpines)

add_library(BigPuppySymmetricSpiral1
	    BigPuppySymmetricSpiral1.cpp)

add_executable(AppBigPuppySymmetricSpiral1
    BigPuppySymmetricSpiral1.cpp
    AppBigPuppySymmetricSpiral1.cpp
) 
