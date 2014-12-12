project(CPG_feedback)

add_library( ${PROJECT_NAME} SHARED
	CPGNodeFB.cpp
	CPGEquationsFB.cpp
    SpineFeedbackControl.cpp
)

link_directories(${LIB_DIR})

target_link_libraries(${PROJECT_NAME} KinematicString learningSpines util core)

