project(sensors)

link_directories(${LIB_DIR})

link_libraries(${PROJECT_NAME} util core tgOpenGLSupport)

add_library( ${PROJECT_NAME} SHARED
  # Older software
  tgDataLogger.cpp
  tgDataObserver.cpp

  # For the new sensors
  tgDataManager.cpp
  tgDataLogger2.cpp
    
  tgSensor.cpp
  tgRodSensor.cpp
  tgSpringCableActuatorSensor.cpp
  
  tgSensorInfo.cpp
  tgRodSensorInfo.cpp
  tgSpringCableActuatorSensorInfo.cpp
)
