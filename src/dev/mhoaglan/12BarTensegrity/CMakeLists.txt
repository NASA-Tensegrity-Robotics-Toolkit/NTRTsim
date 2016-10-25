link_directories(${LIB_DIR})

link_libraries(core
               tgcreator
               controllers
               util
               sensors
               terrain
               tgOpenGLSupport
               yaml-cpp
	       Adapters
	       TensegrityModel)

# add_library(TensegrityModel
#     TensegrityModel.cpp)

add_executable(App12Bar1
#    TensegrityModel.cpp
    App12Bar1.cpp
    RPLengthController.h
    RPLengthController_rand.cpp)


