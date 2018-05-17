#include <ros/ros.h>
#include "LidarStopTrigger.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_stop_trigger");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  lidar_stop_trigger::LidarStopTrigger node(n, pn);

  ros::spin();
}