#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub_twist_cmd;
double stop_line_dist;

void recvDist(const std_msgs::Float64ConstPtr& msg)
{
  stop_line_dist = msg->data;
}

void timerCallback(const ros::TimerEvent& event)
{
  geometry_msgs::Twist cmd;

  if (stop_line_dist < 6.0) {
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
  } else {
    cmd.linear.x = 2.0;
    cmd.angular.z = 0.0;
  }
  pub_twist_cmd.publish(cmd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stop_line_test");
  ros::NodeHandle n;

  pub_twist_cmd = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber sub_stop_trigger = n.subscribe("stop_line_dist", 1, recvDist);

  ros::Timer timer = n.createTimer(ros::Duration(0.02), &timerCallback);

  stop_line_dist = INFINITY;
  ros::spin();
}
