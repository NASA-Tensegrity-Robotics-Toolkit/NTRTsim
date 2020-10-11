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

add_executable(AppBelkaWalking
  BelkaWalkingController.cpp
  BelkaWalkingFileController.cpp
  BelkaWalkingModel.cpp
  AppBelkaWalking.cpp
)
