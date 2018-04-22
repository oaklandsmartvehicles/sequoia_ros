#include <sequoia_gazebo/SequoiaInterfacePlugin.h>

namespace gazebo
{

SequoiaInterfacePlugin::SequoiaInterfacePlugin()
{
  target_angle_ = 0.0;
  brake_cmd_ = 0.0;
  throttle_cmd_ = 0.0;
  rollover_ = false;
}

void SequoiaInterfacePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Gazebo initialization
  left_steer_joint_ = model->GetJoint("steer_fl");
  right_steer_joint_ = model->GetJoint("steer_fr");
  wheel_rl_joint_ = model->GetJoint("wheel_rl");
  wheel_rr_joint_ = model->GetJoint("wheel_rr");
  wheel_fl_joint_ = model->GetJoint("wheel_fl");
  wheel_fr_joint_ = model->GetJoint("wheel_fr");
  model_ = model;
  footprint_link_ = model->GetLink("base_footprint");

  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&SequoiaInterfacePlugin::OnUpdate, this, _1));

#if GAZEBO_MAJOR_VERSION > 2
  right_steer_joint_->SetParam("fmax", 0, 99999.0);
  left_steer_joint_->SetParam("fmax", 0, 99999.0);
#else
  right_steer_joint_->SetMaxForce(0, 99999.0);
  left_steer_joint_->SetMaxForce(0, 99999.0);
#endif

  // ROS initialization
  n_ = new ros::NodeHandle(model->GetName());

  sub_steering_cmd_ = n_->subscribe("steering_cmd", 1, &SequoiaInterfacePlugin::recvSteeringCmd, this);
  sub_brake_cmd_ = n_->subscribe("brake_cmd", 1, &SequoiaInterfacePlugin::recvBrakeCmd, this);
  sub_throttle_cmd_ = n_->subscribe("throttle_cmd", 1, &SequoiaInterfacePlugin::recvThrottleCmd, this);

  pub_twist_ = n_->advertise<geometry_msgs::TwistStamped>("twist", 1);

  twist_timer_ = n_->createTimer(ros::Duration(0.02), &SequoiaInterfacePlugin::twistTimerCallback, this);
}

void SequoiaInterfacePlugin::OnUpdate(const common::UpdateInfo& info)
{
  steeringUpdate();
  driveUpdate();

  twist_.linear.x = footprint_link_->GetRelativeLinearVel().x;
  twist_.angular.z = footprint_link_->GetRelativeAngularVel().z;

  // Detect rollovers
  math::Pose global_pose = footprint_link_->GetWorldPose();
  rollover_ = (fabs(global_pose.rot.x) > 0.2 || fabs(global_pose.rot.y) > 0.2);
}

void SequoiaInterfacePlugin::driveUpdate()
{
  // Stop wheels if vehicle is rolled over
  if (rollover_) {
    wheel_rl_joint_->SetForce(0, -1000.0 * wheel_rl_joint_->GetVelocity(0));
    wheel_rr_joint_->SetForce(0, -1000.0 * wheel_rr_joint_->GetVelocity(0));
    wheel_fl_joint_->SetForce(0, -1000.0 * wheel_fl_joint_->GetVelocity(0));
    wheel_fr_joint_->SetForce(0, -1000.0 * wheel_fr_joint_->GetVelocity(0));
    return;
  }

  // Brakes have precedence over throttle
  ros::Time current_stamp = ros::Time::now();
  if ((brake_cmd_ > 0) && ((current_stamp - brake_stamp_).toSec() < 0.25)) {
    double brake_adjust_factor = 1.0;
    if (twist_.linear.x < -0.1) {
      brake_adjust_factor = -1.0;
    } else if (twist_.linear.x < 0.1) {
      brake_adjust_factor = 1.0 + (twist_.linear.x - 0.1) / 0.1;
    }

    setWheelTorque(-brake_adjust_factor * brake_cmd_);
  } else {
    if ((current_stamp - throttle_stamp_).toSec() < 0.25) {
      // Sigmoid function approximating torque dropoff as max speed is reached
      double throttle_torque = throttle_cmd_ * 500.0 * (1 - 1 / (1 + exp(-1.5 * (twist_.linear.x - 9.7))));
      if (throttle_torque < 0.0) {
        throttle_torque = 0.0;
      }
      setWheelTorque(throttle_torque);
    }
  }

  // Rolling resistance
  double rolling_resistance_torque = ROLLING_RESISTANCE_COEFF * MASS * G;
  if (twist_.linear.x > 0.0) {
    setWheelTorque(-rolling_resistance_torque);
  } else {
    setWheelTorque(rolling_resistance_torque);
  }
}

void SequoiaInterfacePlugin::setWheelTorque(double torque)
{
  wheel_rl_joint_->SetForce(0, 0.25 * torque);
  wheel_rr_joint_->SetForce(0, 0.25 * torque);
  wheel_fl_joint_->SetForce(0, 0.25 * torque);
  wheel_fr_joint_->SetForce(0, 0.25 * torque);
}

void SequoiaInterfacePlugin::steeringUpdate()
{
  // Compute Ackermann steering angles for each wheel
  double t_alph = tan(target_angle_);
  double left_steer = atan(WHEELBASE * t_alph / (WHEELBASE - 0.5 * TRACK_WIDTH * t_alph));
  double right_steer = atan(WHEELBASE * t_alph / (WHEELBASE + 0.5 * TRACK_WIDTH * t_alph));

  // Compute appropriate steering velocities and apply to joints
  double right_steer_vel = 10.0 * (right_steer - right_steer_joint_->GetAngle(0).Radian());
  if (right_steer_vel > 0.75) {
    right_steer_vel = 0.75;
  } else if (right_steer_vel < -0.75) {
    right_steer_vel = -0.75;
  }

#if GAZEBO_MAJOR_VERSION > 2
  right_steer_joint_->SetParam("vel", 0, right_steer_vel);
#else
  right_steer_joint_->SetVelocity(0, right_steer_vel);
#endif

  double left_steer_vel = 10.0 * (left_steer - left_steer_joint_->GetAngle(0).Radian());
  if (left_steer_vel > 0.75) {
    left_steer_vel = 0.75;
  } else if (left_steer_vel < -0.75) {
    left_steer_vel = -0.75;
  }

#if GAZEBO_MAJOR_VERSION > 2
  left_steer_joint_->SetParam("vel", 0, left_steer_vel);
#else
  left_steer_joint_->SetVelocity(0, left_steer_vel);
#endif
}

void SequoiaInterfacePlugin::recvSteeringCmd(const std_msgs::Float64ConstPtr& msg)
{
  if (!std::isfinite(msg->data)) {
    ROS_WARN_THROTTLE(0.5, "Steering command is NaN!");
    target_angle_ = 0.0;
    return;
  }

  target_angle_ = msg->data / STEERING_RATIO;
  if (target_angle_ > MAX_STEERING_ANGLE) {
    target_angle_ = MAX_STEERING_ANGLE;
  } else if (target_angle_ < -MAX_STEERING_ANGLE) {
    target_angle_ = -MAX_STEERING_ANGLE;
  }
}

void SequoiaInterfacePlugin::recvBrakeCmd(const std_msgs::Float64ConstPtr& msg)
{
  brake_cmd_ = msg->data;
  if (brake_cmd_ < 0) {
    brake_cmd_ = 0;
  } else if (brake_cmd_ > MAX_BRAKE_TORQUE) {
    brake_cmd_ = MAX_BRAKE_TORQUE;
  }
  brake_stamp_ = ros::Time::now();
}

void SequoiaInterfacePlugin::recvThrottleCmd(const std_msgs::Float64ConstPtr& msg)
{
  throttle_cmd_ = msg->data;
  if (throttle_cmd_ < 0.0) {
    throttle_cmd_ = 0.0;
  } else if (throttle_cmd_ > 1.0) {
    throttle_cmd_ = 1.0;
  }
  throttle_stamp_ = ros::Time::now();
}

void SequoiaInterfacePlugin::twistTimerCallback(const ros::TimerEvent& event)
{
  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header.frame_id = "base_footprint";
  twist_msg.header.stamp = event.current_real;
  twist_msg.twist = twist_;
  pub_twist_.publish(twist_msg);
}

void SequoiaInterfacePlugin::Reset()
{
}

SequoiaInterfacePlugin::~SequoiaInterfacePlugin()
{
  n_->shutdown();
  delete n_;
}


}
