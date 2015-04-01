Project(obstacles)

link_directories(${LIB_DIR})


link_libraries(tgcreator
                core
                terrain
                tgOpenGLSupport)

add_library(${PROJECT_NAME} SHARED
			tgBlockField.cpp
        		tgStairs.cpp
			tgCraterDeep.cpp
			tgCraterShallow.cpp
            )

add_executable(AppObstacleTest
	tgBlockField.cpp
    tgStairs.cpp
	AppObstacleTest.cpp
)

