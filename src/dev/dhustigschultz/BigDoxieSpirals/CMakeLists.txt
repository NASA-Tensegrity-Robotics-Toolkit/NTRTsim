link_libraries(tgcreator 
	       core
	       learningSpines)

add_library(BigDoxieSpirals
	    BigDoxieSpirals.cpp)

add_executable(AppBigDoxieSpirals
    BigDoxieSpirals.cpp
    AppBigDoxieSpirals.cpp
) 
