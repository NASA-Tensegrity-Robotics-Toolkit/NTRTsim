link_directories(${LIB_DIR})

link_libraries(core
               tgcreator
               util
               sensors
               terrain
               tgOpenGLSupport
               yaml-cpp
)

# Two things to build here.
# First, build TensegrityModel as a library.
# This file is the YAMLbuilder itself, and can be used
# in other App files to bring in a YAML model.
add_library(TensegrityModel
    TensegrityModel.cpp
)

# Additionally, the BuildModel App provides a template
# for using TensegrityModel.
add_executable(BuildModel
    TensegrityModel.cpp
    BuildTensegrityModel.cpp
)


