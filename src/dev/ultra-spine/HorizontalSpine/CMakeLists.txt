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

# add_library(TensegrityModel
#     TensegrityModel.cpp)

add_executable(AppHorizontalSpine
#    TensegrityModel.cpp
    AppHorizontalSpine.cpp)


