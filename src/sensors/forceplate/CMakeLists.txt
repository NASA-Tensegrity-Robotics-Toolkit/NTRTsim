# This project will be specifically for the force plate.
#project(sensors)

# Used to be:
project(forceplate)

#link_directories(${LIB_DIR})

# Used to be:
#link_libraries(tgcreator util core tgOpenGLSupport)
link_libraries(${LIB_DIR} tgcreator util core tgOpenGLSupport)

# The force plate library has only one file: the force plate model.
#add_library( ${PROJECT_NAME} SHARED
#add_executable(

# Used to be:
add_library( ${PROJECT_NAME} SHARED
  ForcePlateModel.cpp
  ForcePlateSensor.cpp
)

  
