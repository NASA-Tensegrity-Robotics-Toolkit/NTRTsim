project(sensors)

# specify the force plate as a separate library.
subdirs(
  forceplate
)

link_directories(${LIB_DIR})

link_libraries(${PROJECT_NAME} util core tgOpenGLSupport)

add_library( ${PROJECT_NAME} SHARED
    tgDataLogger.cpp
    tgDataObserver.cpp
    # The ForcePlateModel and sensor are also needed here,
    # since the data logger needs to know how to render a force plate.
    forceplate/ForcePlateModel.cpp
    forceplate/ForcePlateSensor.cpp
)
#forceplate/ForcePlateModel.cpp
  
