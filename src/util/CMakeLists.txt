project(util)

add_library( ${PROJECT_NAME} SHARED
	CPGNode.cpp
	CPGEquations.cpp
    tgBaseCPGNode.cpp
)

link_directories(${LIB_DIR})

target_link_libraries(${PROJECT_NAME} controllers core)

