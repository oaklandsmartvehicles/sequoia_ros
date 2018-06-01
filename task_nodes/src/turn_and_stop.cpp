#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

ros::Publisher pub_twist_cmd;
ros::Publisher pub_move_vehicle_mode;
tf::TransformListener* listener;

// Inputs
double stop_line_dist;
bool lidar_stop_trigger;
geometry_msgs::Twist move_base_cmd;
double start_heading;

// Parameters
double sensitive_range;
double angle_change_thres;
std_msgs::Int8 turn_mode_msg;

enum {LANE_KEEP_1=0, TURN, LANE_KEEP_2, STOP};
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

  tf::StampedTransform transform;
  try {
    listener->lookupTransform("map", "base_footprint", ros::Time(0), transform);
  } catch (tf::TransformException& ex) {
    ROS_WARN_THROTTLE(1.0, "%s", ex.what());
    return;
  }

  switch (state) {
    case LANE_KEEP_1:
      cmd = move_base_cmd;

      if (stop_line_dist < sensitive_range) {
        state = TURN;
        pub_move_vehicle_mode.publish(turn_mode_msg);

        start_heading = atan2(2 * transform.getRotation().w() * transform.getRotation().z(),
                              1 - 2 * transform.getRotation().z() * transform.getRotation().z());
      }
      break;
    case TURN: {
      cmd = move_base_cmd;

      double heading = atan2(2 * transform.getRotation().w() * transform.getRotation().z(),
                             1 - 2 * transform.getRotation().z() * transform.getRotation().z());

      double delta = heading - start_heading;
      if (delta > M_PI) {
        delta -= 2 * M_PI;
      } else if (delta < -M_PI) {
        delta += 2 * M_PI;
      }

      if (fabs(delta) > angle_change_thres) {
        state = LANE_KEEP_2;
      }
    }
      break;
    case LANE_KEEP_2:
      cmd = move_base_cmd;

      if (lidar_stop_trigger) {
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
  ros::init(argc, argv, "turn_and_stop");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  pub_twist_cmd = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pub_move_vehicle_mode = n.advertise<std_msgs::Int8>("move_vehicle_mode", 1, true);
  ros::Subscriber sub_stop_trigger = n.subscribe("stop_trigger", 1, recvStopTrigger);
  ros::Subscriber sub_stop_line_dist = n.subscribe("stop_line_dist", 1, recvDist);
  ros::Subscriber sub_move_base_twist = n.subscribe("move_base_cmd_vel", 1, recvCmdVel);

  ros::Timer timer = n.createTimer(ros::Duration(0.02), &timerCallback);

  listener = new tf::TransformListener;

  pn.param("sensitive_range", sensitive_range, 4.1);
  pn.param("angle_change_thres", angle_change_thres, 1.4);
  std::string turn_direction;
  pn.param("turn_direction", turn_direction, std::string("left"));

  if (turn_direction == "left") {
    turn_mode_msg.data = 1;
  } else if (turn_direction == "right") {
    turn_mode_msg.data = 2;
  } else {
    ROS_WARN("Invalid turn direction [%s]", turn_direction.c_str());
  }

  stop_line_dist = INFINITY;
  lidar_stop_trigger = false;
  state = LANE_KEEP_1;

  std_msgs::Int8 mode_msg;
  mode_msg.data = 0; // Straight
  pub_move_vehicle_mode.publish(mode_msg);

  ros::spin();
}
