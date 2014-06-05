project(models)

add_library( ${PROJECT_NAME} SHARED
NestedStructureTestModel.cpp
T6Model.cpp
)

link_directories(${LIB_DIR})

target_link_libraries(${PROJECT_NAME} tgcreator core)

