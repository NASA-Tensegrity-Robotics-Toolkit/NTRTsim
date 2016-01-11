link_libraries(tgcreator 
	       core
	       learningSpines)

add_library(BigPuppyNoFeet
	    BigPuppyNoFeet.cpp)

add_executable(AppBigPuppyNoFeet
    BigPuppyNoFeet.cpp
    AppBigPuppyNoFeet.cpp) 
