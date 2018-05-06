// Header file for the class
#include "DeadReckoning.h"

// Namespace matches ROS package name
namespace dead_reckoning {

// Constructor with global and private node handle arguments
DeadReckoning::DeadReckoning(ros::NodeHandle n, ros::NodeHandle pn)
{

  sub_twist = n.subscribe("/vehicle/twist", 1, &DeadReckoning::recvTwist, this);
  
  pn.param("parent_frame", parent_frame, std::string("odom"));
  pn.param("child_frame", child_frame, std::string("base_footprint"));
  
  x = 0;
  y = 0;
  psi = 0;
  sample_time = 0.02;
  
  timer = n.createTimer(ros::Duration(sample_time), &DeadReckoning::timerCallback, this);
}

void DeadReckoning::timerCallback(const ros::TimerEvent& event)
{
  double v = twist_data.twist.linear.x;
  double pdot = twist_data.twist.angular.z;
  
  
  
  x += sample_time * v * cos(psi);
  y = y + sample_time * v * sin(psi);
  psi = psi + sample_time * pdot;
  
  tf::StampedTransform transform;
  transform.frame_id_ = parent_frame;
  transform.child_frame_id_ = child_frame;
  transform.stamp_ = event.current_real;
  transform.setOrigin(tf::Vector3(x, y, 0));
  transform.setRotation(tf::createQuaternionFromYaw(psi));
  broadcaster.sendTransform(transform);
  
}

void DeadReckoning::recvTwist(const geometry_msgs::TwistStampedConstPtr& msg)
{
  twist_data = *msg;
}


  
}
