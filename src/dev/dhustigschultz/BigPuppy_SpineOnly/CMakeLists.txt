link_libraries(tgcreator 
	       core
	       learningSpines)

add_library(BigPuppySpineOnly
	    BigPuppySpineOnly.cpp)

add_executable(AppBigPuppySpineOnly
    BigPuppySpineOnly.cpp
    AppBigPuppySpineOnly.cpp
) 
