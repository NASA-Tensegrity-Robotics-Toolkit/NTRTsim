link_libraries(
    core
    tgcreator
)

link_directories(${LIB_DIR})


include_directories(${INC_DIR})

add_executable(AppTgCreatorTest
    main.cpp
) 

add_executable(AppNestedStructureTest
    AppNestedStructureTest.cpp
)