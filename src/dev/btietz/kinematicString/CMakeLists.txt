project(KinematicString)

add_library( ${PROJECT_NAME} SHARED
	tgSCASineControl.cpp
    tgCPGCableControl.cpp
)

link_directories(${LIB_DIR})

target_link_libraries(${PROJECT_NAME} learningSpines controllers core)

