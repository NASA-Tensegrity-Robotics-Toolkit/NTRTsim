link_libraries(tgcreator core)

add_library(MuscleNPCons SHARED
			MuscleNPCons.cpp)

add_executable(AppMuscleNPCons
    MuscleNPCons.cpp
    AppMuscleNPCons.cpp
) 
