#include <ros/ros.h>
#include "SequoiaNavFilter.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sequoia_nav_filter");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  sequoia_nav_filter::SequoiaNavFilter node(n, pn);
  
  ros::spin();
}