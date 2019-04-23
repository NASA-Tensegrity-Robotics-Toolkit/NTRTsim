link_directories(${LIB_DIR} boost_system)

link_libraries(core
               tgcreator
               util
               sensors
               terrain
               tgOpenGLSupport
               yaml-cpp
               Adapters
               TensegrityModel)

# add_library(TensegrityModel
#     TensegrityModel.cpp)

add_executable(AppLaikaVideos
	AppLaikaVideos.cpp
     ../v0.2_combined/CombinedSpineControllerBending.cpp
)


