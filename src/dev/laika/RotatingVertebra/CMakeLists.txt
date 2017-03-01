link_directories(${LIB_DIR})

link_libraries(core
               tgcreator
               util
               sensors
               terrain
               tgOpenGLSupport
               yaml-cpp
	       Adapters
	       TensegrityModel
	       forceplate)

# add_library(TensegrityModel
#     TensegrityModel.cpp)

add_executable(AppRotatingVertebraSpine
  RotatingVertebraController.cpp
  HorizontalSpineController.cpp
  AppRotatingVertebraSpine.cpp
)


