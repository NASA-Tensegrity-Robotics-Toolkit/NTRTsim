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

add_executable(AppLaikaCombinedMotion
  CombinedSpineControllerRotVertPositionTraj.cpp
  CombinedSpineControllerBending.cpp
  AppLaikaCombinedMotion.cpp
)


