// ROS and node class header file
#include <ros/ros.h>
#include "EkfExample.h"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "ekf_example");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  // Instantiate node class
  ekf_example::EkfExample node(n, pn);

  // Spin and process callbacks
  ros::spin();
}
