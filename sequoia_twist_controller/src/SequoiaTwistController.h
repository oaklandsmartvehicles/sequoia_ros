#ifndef SEQUOIATWISTCONTROLLER_H
#define SEQUOIATWISTCONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/TwistStamped.h>

#include <dynamic_reconfigure/server.h>
#include <sequoia_twist_controller/SequoiaTwistConfig.h>

namespace sequoia_twist_controller
{

#define STEERING_RATIO      17.0
#define WHEELBASE           2.4
#define MAX_BRAKE_TORQUE    1000.0 // N-m
#define MAX_STEERING_ANGLE  0.5617  // min turning radius = 3.81 m

class SequoiaTwistController
{
  public:
    SequoiaTwistController(ros::NodeHandle n, ros::NodeHandle pn);

  private:
    void reconfig(SequoiaTwistConfig& config, uint32_t level);
    void timerCallback(const ros::TimerEvent& event);
    void recvTwistCmd(const geometry_msgs::TwistConstPtr& msg);
    void recvTwistActual(const geometry_msgs::TwistStampedConstPtr& msg);
    void recvGearState(const std_msgs::UInt8ConstPtr& msg);

    ros::Publisher pub_throttle_cmd_;
    ros::Publisher pub_brake_cmd_;
    ros::Publisher pub_steering_cmd_;
    ros::Subscriber sub_twist_cmd_;
    ros::Subscriber sub_twist_actual_;
    ros::Subscriber sub_gear_state_;
    ros::Timer control_timer_;

    dynamic_reconfigure::Server<SequoiaTwistConfig> srv_;
    SequoiaTwistConfig cfg_;

    geometry_msgs::TwistStamped twist_cmd_;
    geometry_msgs::TwistStamped twist_actual_;
    int gear_state_; // 0 = Neutral, 1 = Forward, 2 = Reverse
    double int_throttle_;
};

}

#endif // SEQUOIATWISTCONTROLLER_H
