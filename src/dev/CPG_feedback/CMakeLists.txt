project(CPG_feedback)

add_library( ${PROJECT_NAME} SHARED
	CPGNodeFB.cpp
	CPGEquationsFB.cpp
)

link_directories(${LIB_DIR})

target_link_libraries(${PROJECT_NAME} util core)

