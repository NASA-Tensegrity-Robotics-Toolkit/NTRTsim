link_directories(${LIB_DIR})


link_libraries(tgcreator
                core
                terrain
                tgOpenGLSupport)

add_executable(AppObstacleTest
	tgBlockField.cpp
	AppObstacleTest.cpp
)

