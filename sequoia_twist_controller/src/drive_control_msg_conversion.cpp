#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dbw_gem_msgs/ThrottleCmd.h>
#include <dbw_gem_msgs/BrakeCmd.h>
#include <dbw_gem_msgs/SteeringCmd.h>
#include "DiscreteTf.h"

ros::Publisher pub_throttle;
ros::Publisher pub_brake;
ros::Publisher pub_steering;

void recvBrake(const std_msgs::Float64ConstPtr& msg)
{
  // Generate dbw_gem_msgs::BrakeCmd and publish here
  dbw_gem_msgs::BrakeCmd ds_brake_msg;
  ds_brake_msg.enable = true;
  ds_brake_msg.pedal_cmd_type = dbw_gem_msgs::BrakeCmd::CMD_PERCENT;
  ds_brake_msg.pedal_cmd = msg->data;
  pub_brake.publish(ds_brake_msg);
}

void recvThrottle(const std_msgs::Float64::ConstPtr& msg)
{
  // Generate dbw_gem_msgs::ThrottleCmd and publish here
  dbw_gem_msgs::ThrottleCmd ds_throttle_cmd;
  ds_throttle_cmd.enable = true;
  ds_throttle_cmd.pedal_cmd_type = dbw_gem_msgs::ThrottleCmd::CMD_PERCENT;
  ds_throttle_cmd.pedal_cmd = msg->data;
  pub_throttle.publish(ds_throttle_cmd);

}

void recvSteering(const std_msgs::Float64ConstPtr& msg)
{
  // Generate dbw_gem_msgs::SteeringCmd and publish here
   dbw_gem_msgs::SteeringCmd ds_steering_msg;
  ds_steering_msg.enable = true;
  steering_cmd_filter = (0.97 * steering_cmd_filter) +(0.03 * msg->data);
  ds_steering_msg.steering_wheel_angle_cmd = steering_cmd_filter;
  pub_steering.publish(ds_steering_msg);
  

}
/*void timerCallback(const ros::TimerEvent& event)
{
 
   dbw_gem_msgs::SteeringCmd ds_steering_msg;
  ds_steering_msg.enable = true;
   steering_cmd_filter = (0.97 * steering_cmd_filter) +(0.03 * msg->data);
  steering_msg.steering_wheel_angle_cmd = steering_cmd_filter;
  pub_steering.publish(ds_steering_msg);
}*/
/*void recvGearState(const std_msgs::UInt8ConstPtr& msg)
{
  dbw_gem_msgs::BrakeCmd ds_brake_msg;
  ds_brake_msg.gear = msg->data;  
  pub_brake.publish(ds_brake_msg);
}*/

int main(int argc, char** argv)
{
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pub_brake = n.advertise<dbw_gem_msgs::BrakeCmd>("brake_cmd", 1);
  pub_throttle = n.advertise<dbw_gem_msgs::ThrottleCmd>("throttle_cmd", 1);
  pub_steering = n.advertise<dbw_gem_msgs::SteeringCmd>("steering_cmd", 1);

 // ros::Subscriber sub_Gear = n.subscribe("Stc_gear_state", 1, recvGearState);
  ros::Subscriber sub_brake = n.subscribe("Stc_brake_cmd", 1, recvBrake);
  ros::Subscriber sub_throttle = n.subscribe("Stc_throttle_cmd", 1, recvThrottle);
  ros::Subscriber sub_steering = n.subscribe("Stc_steering_cmd", 1, recvSteering);


  //ros::Timer timer_s = n.createTimer(ros::Duration(0.02), timerCallback);


  ros::spin();
}
