project(controllers)

add_library( ${PROJECT_NAME} SHARED
PretensionController.cpp
NestedStructureSineWaves.cpp
T6TensionController.cpp
)

link_directories(${LIB_DIR})

target_link_libraries(${PROJECT_NAME} models core)

