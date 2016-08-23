# This project will be specifically for the force plate.
#project(sensors)
project(forceplate)

link_directories(${LIB_DIR})

link_libraries(${PROJECT_NAME} util core tgOpenGLSupport)

# The force plate library has only one file: the force plate model.
add_library( ${PROJECT_NAME} SHARED
    ForcePlateModel.cpp
)

  
