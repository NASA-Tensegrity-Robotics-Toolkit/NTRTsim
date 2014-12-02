link_libraries(tgcreator MuscleNP core)

add_library(MuscleNPCons SHARED
			MuscleNPCons.cpp)

add_executable(AppMuscleNPCons
    MuscleNPCons.cpp
    AppMuscleNPCons.cpp
) 
