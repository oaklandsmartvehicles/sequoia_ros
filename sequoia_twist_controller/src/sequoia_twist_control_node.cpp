#include <ros/ros.h>
#include "SequoiaTwistController.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sequoia_twist_control");

  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  sequoia_twist_controller::SequoiaTwistController node(n, pn);

  ros::spin();
}
