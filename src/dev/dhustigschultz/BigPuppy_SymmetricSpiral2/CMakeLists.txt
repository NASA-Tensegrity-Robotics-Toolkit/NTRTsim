link_libraries(tgcreator 
	       core
	       learningSpines)

add_library(BigPuppySymmetricSpiral2
	    BigPuppySymmetricSpiral2.cpp)

add_executable(AppBigPuppySymmetricSpiral2
    BigPuppySymmetricSpiral2.cpp
    AppBigPuppySymmetricSpiral2.cpp
) 
