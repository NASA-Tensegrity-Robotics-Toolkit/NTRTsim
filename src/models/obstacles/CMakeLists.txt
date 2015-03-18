Project(obstacles)

link_directories(${LIB_DIR})


link_libraries(tgcreator
                core
                terrain
                tgOpenGLSupport)

add_library(${PROJECT_NAME} SHARED
			tgBlockField.cpp
            tgStairs.cpp
            )

add_executable(AppObstacleTest
	tgBlockField.cpp
    tgStairs.cpp
	AppObstacleTest.cpp
)

