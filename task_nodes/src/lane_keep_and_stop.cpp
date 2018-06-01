#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub_twist_cmd;

// Inputs
double stop_line_dist;
bool lidar_stop_trigger;
geometry_msgs::Twist move_base_cmd;

// Parameters
double sensitive_range;

enum {LANE_KEEP=0, STOP=1};
int state;

void recvDist(const std_msgs::Float64ConstPtr& msg)
{
  stop_line_dist = msg->data;
}

void recvStopTrigger(const std_msgs::BoolConstPtr& msg)
{
  lidar_stop_trigger = msg->data;
}

void recvCmdVel(const geometry_msgs::TwistConstPtr& msg)
{
  move_base_cmd = *msg;
}

void timerCallback(const ros::TimerEvent& event)
{
  geometry_msgs::Twist cmd;

  switch (state) {
    case LANE_KEEP:
      cmd = move_base_cmd;

      if (stop_line_dist < sensitive_range || lidar_stop_trigger) {
        state = STOP;
      }
      break;
    case STOP:
      break;
  }

  pub_twist_cmd.publish(cmd);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "stop_line_test");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pub_twist_cmd = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber sub_stop_trigger = n.subscribe("stop_trigger", 1, recvStopTrigger);
  ros::Subscriber sub_stop_line_dist = n.subscribe("stop_line_dist", 1, recvDist);
  ros::Subscriber sub_move_base_twist = n.subscribe("move_base_cmd_vel", 1, recvCmdVel);

  ros::Timer timer = n.createTimer(ros::Duration(0.02), &timerCallback);

  pn.param("sensitive_range", sensitive_range, 6.0);

  stop_line_dist = INFINITY;
  lidar_stop_trigger = false;
  state = LANE_KEEP;
  ros::spin();
}
