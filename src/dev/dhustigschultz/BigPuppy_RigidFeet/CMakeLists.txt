link_libraries(tgcreator 
	       core
	       learningSpines)

add_library(BigPuppyRigidFeet
	    BigPuppyRigidFeet.cpp)

add_executable(AppBigPuppyRigidFeet
    BigPuppyRigidFeet.cpp
    AppBigPuppyRigidFeet.cpp) 
