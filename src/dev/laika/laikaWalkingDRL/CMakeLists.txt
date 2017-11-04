link_directories(${LIB_DIR})

link_libraries(core
               tgcreator
               util
               sensors
               terrain
               tgOpenGLSupport
               yaml-cpp
	       Adapters
	       TensegrityModel)

add_executable(AppLaikaWalkingDRL
  HorizontalSpineController.cpp
  LaikaWalkingModel.cpp
  AppLaikaWalkingDRL.cpp
)
