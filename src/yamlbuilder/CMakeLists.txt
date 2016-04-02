link_directories(${LIB_DIR})

link_libraries(core
               learningSpines
               tgcreator
               util
               sensors
               terrain
               tgOpenGLSupport
               yaml-cpp)

add_library(TensegrityModel
    TensegrityModel.cpp)

add_executable(BuildModel
    TensegrityModel.cpp
    BuildTensegrityModel.cpp)


