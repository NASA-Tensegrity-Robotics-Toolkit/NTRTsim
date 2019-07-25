link_directories(${LIB_DIR})

link_libraries(core
	tgcreator
	util
	sensors
	terrain
	tgOpenGLSupport
	yaml-cpp
	TensegrityModel)

add_executable(AppSBTest
	StepwiseController.cpp
  SingleCableController.cpp
	AppSBTest.cpp
	)
