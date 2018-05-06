// Include guard to prevent multiple declarations
#ifndef DEADRECKONING_H
#define DEADRECKONING_H

// ROS header
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>


// Namespace matches ROS package name
namespace dead_reckoning{

class DeadReckoning
{
public:
  DeadReckoning(ros::NodeHandle n, ros::NodeHandle pn);
  
private:
  void timerCallback(const ros::TimerEvent& event);
  void recvTwist(const geometry_msgs::TwistStampedConstPtr& msg);
  
  ros::Subscriber sub_twist;
  ros::Timer timer;
  
  geometry_msgs::TwistStamped twist_data;
  
  tf::TransformBroadcaster broadcaster;
  
  double x;
  double y;
  double psi;
  double sample_time;
  
  std::string parent_frame;
  std::string child_frame;
  
  
};

}

#endif // DEADRECKONING_H

