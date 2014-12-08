project(KinematicString)

add_library( ${PROJECT_NAME} SHARED
	tgKinematicString.cpp
	tgKinematicStringInfo.cpp
	tgBasicController.cpp
	tgPIDController.cpp
)

link_directories(${LIB_DIR})

target_link_libraries(${PROJECT_NAME} core)

