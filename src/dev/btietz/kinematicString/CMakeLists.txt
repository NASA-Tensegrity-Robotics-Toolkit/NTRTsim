project(KinematicString)

add_library( ${PROJECT_NAME} SHARED
    KinematicSpineCPGControl.cpp
	tgSCASineControl.cpp
)

link_directories(${LIB_DIR})

target_link_libraries(${PROJECT_NAME} CPG_feedback learningSpines controllers core)

