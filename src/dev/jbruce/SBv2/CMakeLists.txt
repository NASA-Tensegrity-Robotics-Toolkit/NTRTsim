link_directories(${LIB_DIR})

link_libraries(core
	tgcreator
	util
	sensors
	terrain
	tgOpenGlSupport
	yaml-cpp
	TensegrityModel)

add_executable(AppSBTest
	AppSBTest.cpp
	)
