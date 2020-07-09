link_directories(${LIB_DIR})

link_libraries(core
               tgcreator
               util
               sensors
               terrain
               tgOpenGLSupport
               yaml-cpp
)

add_library(TensegrityModel
    TensegrityModel.cpp
    TensegrityModelController.cpp
)

add_executable(BuildModel
    TensegrityModel.cpp
    BuildTensegrityModel.cpp
    TensegrityModelController.cpp
)


