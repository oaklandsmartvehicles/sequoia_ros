#include <ros/ros.h>
#include "LaneDetection.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_detection_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  sequoia_lane_detection::LaneDetection node(n, pn);

  ros::spin();
}