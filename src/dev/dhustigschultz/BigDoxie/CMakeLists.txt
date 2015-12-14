link_libraries(tgcreator 
	       core
	       learningSpines)

add_library(BigDoxie
	    BigDoxie.cpp)

add_executable(AppBigDoxie
    BigDoxie.cpp
    AppBigDoxie.cpp
) 
