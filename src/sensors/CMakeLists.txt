project(sensors)


link_directories(${LIB_DIR})

# Note that we need to compile in support for boost's regex library
# for use in tgCompoundRigidSensor and its info class.
link_libraries(util core tgOpenGLSupport boost_regex)

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
  tgCompoundRigidSensor.cpp
  
  tgSensorInfo.cpp
  tgRodSensorInfo.cpp
  tgSpringCableActuatorSensorInfo.cpp
  tgCompoundRigidSensorInfo.cpp
)


  
