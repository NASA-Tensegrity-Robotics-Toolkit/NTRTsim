# Library of files that are common between tensegrity projects
# Compiled by Brian Tietz, June 2013

project(Configuration)


# Add a library with the same name as the project. The library will contain all of the 
# files listed along with any files referenced by those files, so you usually only have
# to include the 'main' files in this list. 

add_library( ${PROJECT_NAME} SHARED
    configuration.cpp
)

