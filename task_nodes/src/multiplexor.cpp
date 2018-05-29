#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub_twist_cmd;
ros::Publisher pub_gear_cmd;

bool stop_trigger;
bool stop_sign_trigger;
int mode_trigger;
geometry_msgs::Twist cmd_solid;
geometry_msgs::Twist cmd_dashed;
geometry_msgs::Twist cmd_final;
std_msgs::UInt8 gear;

void recvStopTrigger(const std_msgs::BoolConstPtr& msg)
{
  stop_trigger = msg->data;
}

void recvModeTrigger(const std_msgs::Int8ConstPtr& msg)
{
  mode_trigger = msg->data;
}

void recvStopSignTrigger(const std_msgs::BoolConstPtr& msg)
{
  stop_sign_trigger = msg->data;
}

void recvTwist_solid(const geometry_msgs::TwistConstPtr& msg)
{
  geometry_msgs::Twist velocity;
  
  cmd_solid = *msg;
  
}

void recvTwist_dashed(const geometry_msgs::TwistConstPtr& msg)
{
  geometry_msgs::Twist velocity;
  
  cmd_dashed = *msg;
  
}


void timerCallback(const ros::TimerEvent& event)
{
  
    if (stop_trigger) {
    cmd_final.linear.x = 0.0;
    cmd_final.angular.z = 0.0;
    } else {
	  if(mode_trigger == 1){
	    cmd_final = cmd_solid;
	  }else if(mode_trigger == 2){
	    cmd_final = cmd_dashed;
	  }
	  if (cmd_final.linear.x<=0.0) {
	    gear.data = 2;
	    cmd_final.linear.x = cmd_final.linear.x * -1.0;
	  } else {
	    gear.data = 1;
	    cmd_final.linear.x = cmd_final.linear.x;
	  }
    }
	
   
  pub_gear_cmd.publish<std_msgs::UInt8>(gear);
  pub_twist_cmd.publish(cmd_final);
}

// void timerCallback(const ros::TimerEvent& event)
// {
//   
// 
//   
// }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "multiplexor");
  ros::NodeHandle n;

  pub_twist_cmd = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pub_gear_cmd = n.advertise<std_msgs::UInt8>("gear_cmd", 1);
  ros::Subscriber sub_stop_trigger = n.subscribe("stop_trigger", 1, recvStopTrigger);
  ros::Subscriber sub_mode_trigger = n.subscribe("mode_trigger", 1, recvModeTrigger);
  ros::Subscriber sub_stop_sign_trigger = n.subscribe("stop_sign_trigger", 1, recvStopSignTrigger);
  ros::Subscriber sub_move_base_solid = n.subscribe("cmd_vel_solid", 1, recvTwist_solid);
  ros::Subscriber sub_move_base_dashed = n.subscribe("cmd_vel_dashed", 1, recvTwist_dashed);
  ros::Timer timer = n.createTimer(ros::Duration(0.02), &timerCallback);
  
  //TEST DELETE WHEN DONE
  




//   ros::Timer timer = n.createTimer(ros::Duration(0.08), &timerCallback);

  stop_trigger = false;
  mode_trigger = 1;
  ros::spin();
}