#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub_twist_cmd;
ros::Publisher pub_gear_cmd;

bool stop_trigger;
bool stop_sign_trigger;
geometry_msgs::Twist cmd;
std_msgs::UInt8 gear;

void recvStopTrigger(const std_msgs::BoolConstPtr& msg)
{
  stop_trigger = msg->data;
}

void recvStopTrigger(const std_msgs::BoolConstPtr& msg)
{
  stop_sign_trigger = msg->data;
}

void recvTwist(const geometry_msgs::TwistConstPtr& msg)
{
  geometry_msgs::Twist velocity;
  
  cmd = *msg;
  
  if (stop_trigger) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
  } else {
    
      if (cmd.linear.x<=0.0) {
      
	gear.data = 2;
	cmd.linear.x = cmd.linear.x * -1.0;
	
      } else {
	gear.data = 1;
	cmd.linear.x = cmd.linear.x;
        }
	  
    }
	
   
  pub_gear_cmd.publish<std_msgs::UInt8>(gear);
  pub_twist_cmd.publish(cmd);
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
  ros::Subscriber sub_stop_sign_trigger = n.subscribe("stop_sign_trigger", 1, recvStopSignTrigger);
  ros::Subscriber sub_move_base = n.subscribe("cmd_vel_b", 1, recvTwist);


//   ros::Timer timer = n.createTimer(ros::Duration(0.08), &timerCallback);

  stop_trigger = false;
  ros::spin();
}