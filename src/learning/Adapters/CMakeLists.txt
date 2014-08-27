# Adapters for the evolution algorithms
# Brian Tietz, May 2014

project(Adapters)


# Add a library with the same name as the project. The library will contain all of the 
# files listed along with any files referenced by those files, so you usually only have
# to include the 'main' files in this list.

add_library( ${PROJECT_NAME} SHARED
    AnnealAdapter.cpp
    NeuroAdapter.cpp
)

target_link_libraries(${PROJECT_NAME})

target_link_libraries(Adapters AnnealEvolution NeuroEvolution)

# TODO: Should we add in a pkgconfig file (like env/lib/pkgconfig/bullet.pc)?


