link_libraries(tgcreator 
	       core
	       learningSpines)

add_library(BigDoxieNoFeet
	    BigDoxieNoFeet.cpp)

add_executable(AppBigDoxieNoFeet
    BigDoxieNoFeet.cpp
    AppBigDoxieNoFeet.cpp
) 
