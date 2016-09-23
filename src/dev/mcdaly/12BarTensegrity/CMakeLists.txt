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

add_executable(App12Bar
#    TensegrityModel.cpp
    App12Bar.cpp
    RPLengthController.h
    RPLengthController_rand.cpp)


