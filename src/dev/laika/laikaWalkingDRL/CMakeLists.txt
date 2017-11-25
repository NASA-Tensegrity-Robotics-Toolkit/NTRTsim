#### Added for ROS ####
find_package(catkin REQUIRED COMPONENTS std_msgs geometry_msgs genmsg)
#######################

set(BOOST_HOME_DIR ../../../../env/include/boost)

include_directories(include ${catkin_INCLUDE_DIRS}
  ${BOOST_HOME_DIR}
  ~/gps-spine/src/Laika/devel/include)

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

add_executable(AppLaikaWalkingDRL
  LaikaWalkingController.cpp
  LaikaWalkingModel.cpp
  AppLaikaWalkingDRL.cpp
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
target_link_libraries(AppLaikaWalkingDRL
  /opt/ros/indigo/lib/librostime.so
  /opt/ros/indigo/lib/libroscpp.so
  /opt/ros/indigo/lib/librosconsole.so
  /opt/ros/indigo/lib/libroscpp_serialization.so)

#target_link_libraries(AppLaikaWalkingDRL
#  ${catkin_LIBRARIES}
#  /opt/ros/kinetic/lib/librostime.so
#  /opt/ros/kinetic/lib/libroscpp.so
#  /opt/ros/kinetic/lib/libroscpp_serialization.so)

#######################
