link_libraries(
    core
    tgcreator
)

link_directories(${LIB_DIR})


include_directories(${INC_DIR})

# add_executable(AppTgCreatorTest
#    AppTgCreatorTest.cpp
#) 


add_executable(AppNestedStructureDepthTest
    AppNestedStructureDepthTest.cpp
) 

add_executable(AppStructuresOnStack
    AppStructuresOnStack.cpp
) 
