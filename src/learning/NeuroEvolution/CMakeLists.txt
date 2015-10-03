# Library of files that are common between tensegrity projects
# Compiled by Brian Tietz, June 2013

project(NeuroEvolution)

link_directories(${LIB_DIR})

include_directories(.)

# Add a library with the same name as the project. The library will contain all of the 
# files listed along with any files referenced by those files, so you usually only have
# to include the 'main' files in this list. 

add_library( ${PROJECT_NAME} SHARED
	NeuroEvolution.cpp
	NeuroEvoMember.cpp
	NeuroEvoPopulation.cpp
)

# Note: FileHelpers seems to be necessary, at least for build on mac...
target_link_libraries(NeuroEvolution neuralNetwork Configuration)


