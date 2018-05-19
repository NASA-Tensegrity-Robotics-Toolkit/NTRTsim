#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listenerAPP");

  /* Main access point to communications with the ROS system. */
  ros::NodeHandle n;

  /* subscrie() says we want to receive message on a given topic */ 
  ros::Subscriber sub = n.subscribe("chatterAPP", 100, chatterCallback);

  ros::spin();

  return 0;

}
