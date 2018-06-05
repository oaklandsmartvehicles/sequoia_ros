#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub_twist_cmd;
unsigned int counter= 0;
double stop_time;
// Inputs
double stop_line_dist;
bool lidar_stop_trigger;
geometry_msgs::Twist move_base_cmd;

// Parameters
double sensitve_range;

enum {LANE_KEEP=0, STOP, LANE_KEEP1, STOP1};
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
      ROS_INFO ("LANE_KEEP");
      if ((stop_line_dist < sensitve_range)) {
        state = STOP;
      }
      break;
    case STOP:
      ROS_INFO ("STOP");

      if (counter<stop_time){
      counter++;
      }
      else{
      counter = 0;
      state= LANE_KEEP1;
      }
      break;
    case LANE_KEEP1:
      cmd = move_base_cmd;
      ROS_INFO ("LANE_KEEP1");

      if (lidar_stop_trigger) {
        state = STOP1;
      }
      break;
    case STOP1:
      ROS_INFO ("STOP1");
      if (!lidar_stop_trigger) {
        state = LANE_KEEP1;
      }

      break;

     // if (!lidar_stop_trigger && (stop_line_dist > sensitve_range)) {
       // state = LANE_KEEP;
      //}
      //break;
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

  pn.param("sensitve_range", sensitve_range, 6.0);
  pn.param("stop_time", stop_time, 200.0);


  stop_line_dist = INFINITY;
  lidar_stop_trigger = false;
  state = LANE_KEEP;
  ros::spin();
}
