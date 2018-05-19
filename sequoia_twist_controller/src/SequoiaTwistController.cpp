#include "SequoiaTwistController.h"

namespace sequoia_twist_controller
{

SequoiaTwistController::SequoiaTwistController(ros::NodeHandle n, ros::NodeHandle pn)
{
  srv_.setCallback(boost::bind(&SequoiaTwistController::reconfig, this, _1, _2));

  sub_twist_cmd_ = n.subscribe("cmd_vel", 1, &SequoiaTwistController::recvTwistCmd, this);
  sub_twist_actual_ = n.subscribe("twist", 1, &SequoiaTwistController::recvTwistActual, this);
  sub_gear_state_ = n.subscribe("gear_state", 1, &SequoiaTwistController::recvGearState, this);
  pub_throttle_cmd_ = n.advertise<std_msgs::Float64>("throttle_cmd", 1);
  pub_brake_cmd_ = n.advertise<std_msgs::Float64>("brake_cmd", 1);
  pub_steering_cmd_ = n.advertise<std_msgs::Float64>("steering_cmd", 1);

  control_timer_ = n.createTimer(ros::Duration(0.02), &SequoiaTwistController::timerCallback, this);
}

void SequoiaTwistController::timerCallback(const ros::TimerEvent& event)
{
  // Timeout after 0.25 seconds
  if ((event.current_real - twist_cmd_.header.stamp).toSec() > 0.25) {
    std_msgs::Float64 zero;
    zero.data = 0.0;
    pub_throttle_cmd_.publish(zero);
    pub_brake_cmd_.publish(zero);
    pub_steering_cmd_.publish(zero);
    return;
  }

  // PI control of throttle and brake
  double speed_error = twist_cmd_.twist.linear.x - twist_actual_.twist.linear.x;
  int_throttle_ += 0.02 *  speed_error;
  if (int_throttle_ >= cfg_.max_throttle) {
    int_throttle_ = cfg_.max_throttle;
  } else if (int_throttle_ < 0.0) {
    int_throttle_ = 0.0;
  }

  std_msgs::Float64 throttle_cmd;
  throttle_cmd.data = cfg_.throttle_p * speed_error + cfg_.throttle_i * int_throttle_;
  if (throttle_cmd.data > cfg_.max_throttle) {
    throttle_cmd.data = cfg_.max_throttle;
  } else if (throttle_cmd.data < 0.0) {
    throttle_cmd.data = 0.0;
  }
  pub_throttle_cmd_.publish(throttle_cmd);

  std_msgs::Float64 brake_cmd;
  if (throttle_cmd.data <= 0.0) {
    brake_cmd.data = -cfg_.brake_gain * speed_error;
  } else {
    brake_cmd.data = 0;
  }

  if (brake_cmd.data > cfg_.max_brake_force) {
    brake_cmd.data = cfg_.max_brake_force;
  } else if (brake_cmd.data < 0.0) {
    brake_cmd.data = 0.0;
  }
  pub_brake_cmd_.publish(brake_cmd);

  // Open loop kinematic control of steering
  std_msgs::Float64 steering_cmd;
  if (fabs(twist_actual_.twist.linear.x) < 0.05) {
    steering_cmd.data = 0.0;
  } else {
    if (gear_state_ > 1) { // Reverse
      steering_cmd.data = -STEERING_RATIO * atan(WHEELBASE * twist_cmd_.twist.angular.z / twist_actual_.twist.linear.x);
    } else { // Forward
      steering_cmd.data = STEERING_RATIO * atan(WHEELBASE * twist_cmd_.twist.angular.z / twist_actual_.twist.linear.x);
    }
  }

  pub_steering_cmd_.publish(steering_cmd);
}

void SequoiaTwistController::recvTwistActual(const geometry_msgs::TwistStampedConstPtr& msg)
{
  twist_actual_ = *msg;
}

void SequoiaTwistController::recvTwistCmd(const geometry_msgs::TwistConstPtr& msg)
{
  twist_cmd_.twist = *msg;
  twist_cmd_.header.stamp = ros::Time::now();
}

void SequoiaTwistController::recvGearState(const std_msgs::UInt8ConstPtr& msg)
{
  gear_state_ = msg->data;
}

void SequoiaTwistController::reconfig(SequoiaTwistConfig& config, uint32_t level)
{
  cfg_ = config;
}

}
