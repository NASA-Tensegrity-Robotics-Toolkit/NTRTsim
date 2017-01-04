# This project will be specifically for the force plate.
project(forceplate)

# Apparently we don't need to link_directories... why?
#link_directories(${LIB_DIR})

# instead, we link the libraries here instead of the directories...?
link_libraries(${LIB_DIR} tgcreator util core tgOpenGLSupport)

# Declare the actual ForcePlateModel library.
# Note that there is no executable associated with this class,
# just a library.
add_library( ${PROJECT_NAME} SHARED
  ForcePlateModel.cpp
  ForcePlateSensor.cpp
)

  
