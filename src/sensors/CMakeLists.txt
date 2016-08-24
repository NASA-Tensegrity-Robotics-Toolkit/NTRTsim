project(sensors)

# specify the force plate as a separate library.
#subdirs(
#  forceplate
#)

link_directories(${LIB_DIR})

link_libraries(${PROJECT_NAME} util core tgOpenGLSupport)

# Since we want the force plate to be part of the sensors library,
# specify the force plate files here with their explicit
# subdirectories.
# @TO-DO: better way?
add_library( ${PROJECT_NAME} SHARED
    tgDataLogger.cpp
    tgDataObserver.cpp
)
#forceplate/ForcePlateModel.cpp
  
