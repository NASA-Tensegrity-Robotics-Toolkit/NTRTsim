#### Added for ROS ####
find_package(catkin REQUIRED COMPONENTS std_msgs)
#######################

include_directories(${catkin_INCLUDE_DIRS})

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
)

#### Added for ROS ####
#target_link_libraries(AppLaikaWalkingDRL
#  /opt/ros/indigo/lib/librostime.so
#  /opt/ros/indigo/lib/libroscpp.so
#  /opt/ros/indigo/lib/librosconsole.so
#  /opt/ros/indigo/lib/libroscpp_serialization.so)

target_link_libraries(AppLaikaWalkingDRL
  /opt/ros/kinetic/lib/librostime.so
  /opt/ros/kinetic/lib/libroscpp.so
  /opt/ros/kinetic/lib/librosconsole.so
  /opt/ros/kinetic/lib/libroscpp_serialization.so)
#######################
