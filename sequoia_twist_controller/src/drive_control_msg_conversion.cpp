#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dbw_gem_msgs/ThrottleCmd.h>
#include <dbw_gem_msgs/BrakeCmd.h>
#include <dbw_gem_msgs/SteeringCmd.h>

ros::Publisher pub_throttle;
ros::Publisher pub_brake;
ros::Publisher pub_steering;

void recvBrake(const std_msgs::Float64ConstPtr& msg)
{
  // Generate dbw_gem_msgs::BrakeCmd and publish here
}

void recvThrottle(const std_msgs::Float64ConstPtr& msg)
{
  // Generate dbw_gem_msgs::ThrottleCmd and publish here

}

void recvSteering(const std_msgs::Float64ConstPtr& msg)
{
  // Generate dbw_gem_msgs::SteeringCmd and publish here

}

int main(int argc, char** argv)
{
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pub_brake = n.advertise<dbw_gem_msgs::BrakeCmd>("ds_brake_cmd", 1);
  pub_throttle = n.advertise<dbw_gem_msgs::ThrottleCmd>("ds_throttle_cmd", 1);
  pub_steering = n.advertise<dbw_gem_msgs::SteeringCmd>("ds_steering_cmd", 1);

  ros::Subscriber sub_brake = n.subscribe("brake_cmd", 1, recvBrake);
  ros::Subscriber sub_throttle = n.subscribe("throttle_cmd", 1, recvThrottle);
  ros::Subscriber sub_steering = n.subscribe("steering_cmd", 1, recvSteering);

  ros::spin();
}