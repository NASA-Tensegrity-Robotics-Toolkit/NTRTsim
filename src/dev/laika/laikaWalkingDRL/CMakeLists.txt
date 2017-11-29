#### Added for ROS ####
find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs genmsg)
#######################

find_package(osg)
find_package(osgViewer)
find_package(OpenThreads)
find_package(osgGA)
find_package(osgText)
set(OSG_LIBRARIES ${OSG_LIBRARY} ${OSGVIEWER_LIBRARY} ${OPENTHREADS_LIBRARY} ${OSGGA_LIBRARY} ${OSGTEXT_LIBRARY})
message(status ${OSG_LIBRARIES})
message(status ${OSG_INCLUDE_DIR})

set(BOOST_HOME_DIR ../../../../env/include/boost)

include_directories(system ${OSG_INCLUDE_DIR})
include_directories(include ${catkin_INCLUDE_DIRS}
  ${BOOST_HOME_DIR}
  ~/gps-spine/src/Laika/devel/include
  )

link_directories(${LIB_DIR})

link_libraries(core
               tgcreator
               util
               sensors
               terrain
               tgOpenGLSupport
               yaml-cpp
	             Adapters
	             TensegrityModel)

#add_executable(AppLaikaWalkingDRL
#  AppLaikaWalkingDRL.cpp
#  LaikaWalkingController.cpp
#  LaikaWalkingModel.cpp
#  NeuralNet.cpp
#  RandomShootingMPC.cpp
#  NeuralNetDynamics.cpp
#  NeuralNetPolicy.cpp
#  ElevationSensor.cpp
#)

add_executable(AppLaikaWalkingDRL_osg
  AppLaikaWalkingDRL_osg.cpp
  LaikaWalkingController.cpp
  LaikaWalkingModel.cpp
  NeuralNet.cpp
  RandomShootingMPC.cpp
  NeuralNetDynamics.cpp
  NeuralNetPolicy.cpp
  ElevationSensor.cpp
)

add_executable(AppSTLParser
  world/parser/AppSTLParser.cpp)

#add_executable(AppElevationGrabber
#	world/parser/AppElevationGrabber.cpp)

#add_executable(AppElevationMap
#	world/parser/AppElevationMap.cpp)

#### Added for ROS ####
#target_link_libraries(AppLaikaWalkingDRL
#  /opt/ros/indigo/lib/librostime.so
#  /opt/ros/indigo/lib/libroscpp.so
#  /opt/ros/indigo/lib/librosconsole.so
#  /opt/ros/indigo/lib/libroscpp_serialization.so
#  FileHelpers
#  ${OSG_LIBRARIES}
#  )

target_link_libraries(AppLaikaWalkingDRL_osg
  /opt/ros/indigo/lib/librostime.so
  /opt/ros/indigo/lib/libroscpp.so
  /opt/ros/indigo/lib/librosconsole.so
  /opt/ros/indigo/lib/libroscpp_serialization.so
  FileHelpers
  ${OSG_LIBRARIES}
  )

#target_link_libraries(AppLaikaWalkingDRL
#  ${catkin_LIBRARIES}
#  /opt/ros/kinetic/lib/librostime.so
#  /opt/ros/kinetic/lib/libroscpp.so
#  /opt/ros/kinetic/lib/librosconsole.so
#  /opt/ros/kinetic/lib/libroscpp_serialization.so)

#######################
