link_libraries(tgcreator controllers core)

add_executable(AppNestedStructureTest
    NestedStructureTestModel.cpp
    NestedStructureSineWaves.cpp
    AppNestedTetrahedrons.cpp
    
)

ntrt_add_runme(AppNestedStructureTest DESCRIPTION "Run the nested tetrahedrons demo")
