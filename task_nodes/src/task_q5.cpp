#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub_twist_cmd;
bool stop_sign_trigger;

void recvStopTrigger(const std_msgs::BoolConstPtr& msg)
{
  stop_sign_trigger = msg->data;
}

void timerCallback(const ros::TimerEvent& event)
{
  geometry_msgs::Twist cmd;

  if (stop_sign_trigger) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
  } else {
    cmd.linear.x = 1.0;
    cmd.angular.z = 0.0;
  }
  pub_twist_cmd.publish(cmd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_q5");
  ros::NodeHandle n;

  pub_twist_cmd = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber sub_stop_trigger = n.subscribe("stop_sign_trigger", 1, recvStopTrigger);

  ros::Timer timer = n.createTimer(ros::Duration(0.02), &timerCallback);

  stop_sign_trigger = false;
  ros::spin();
}
