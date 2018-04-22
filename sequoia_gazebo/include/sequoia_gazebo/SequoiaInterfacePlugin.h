#ifndef AUDIBOTINTERFACEPLUGIN_H
#define AUDIBOTINTERFACEPLUGIN_H

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_msgs/ModelStates.h>

namespace gazebo
{

#define STEERING_RATIO            17.0
#define WHEELBASE                 2.4
#define TRACK_WIDTH               1.2
#define MAX_BRAKE_TORQUE          1000.0 // N-m
#define MAX_STEERING_ANGLE        0.5617  // min turning radius = 3.81 m

#define MASS                      584.0
#define G                         9.81
#define ROLLING_RESISTANCE_COEFF  0.01

enum{GEAR_NEUTRAL=0, GEAR_FORWARD, GEAR_REVERSE};

class SequoiaInterfacePlugin : public ModelPlugin
{
  public:
    SequoiaInterfacePlugin();
    virtual ~SequoiaInterfacePlugin();

  protected:
    virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
    virtual void Reset();

  private:
    void twistTimerCallback(const ros::TimerEvent& event);
    void OnUpdate(const common::UpdateInfo& info);
    void recvSteeringCmd(const std_msgs::Float64ConstPtr& msg);
    void recvThrottleCmd(const std_msgs::Float64ConstPtr& msg);
    void recvBrakeCmd(const std_msgs::Float64ConstPtr& msg);
    void recvGearCmd(const std_msgs::UInt8ConstPtr& msg);
    void steeringUpdate();
    void driveUpdate();
    void setWheelTorque(double torque);

    ros::NodeHandle* n_;
    ros::Publisher pub_twist_;
    ros::Publisher pub_gear_state_;
    ros::Subscriber sub_steering_cmd_;
    ros::Subscriber sub_throttle_cmd_;
    ros::Subscriber sub_brake_cmd_;
    ros::Subscriber sub_gear_shift_cmd_;
    ros::Timer twist_timer_;

    physics::ModelPtr model_;
    geometry_msgs::Twist twist_;
    bool rollover_;
    ros::Time last_model_update_stamp_;
    event::ConnectionPtr update_connection_;
    physics::JointPtr left_steer_joint_;
    physics::JointPtr right_steer_joint_;
    physics::JointPtr wheel_rl_joint_;
    physics::JointPtr wheel_rr_joint_;
    physics::JointPtr wheel_fl_joint_;
    physics::JointPtr wheel_fr_joint_;
    physics::LinkPtr footprint_link_;

    // Steering values
    double right_angle_;
    double left_angle_;
    double target_angle_;

    // Brakes
    double brake_cmd_;
    ros::Time brake_stamp_;

    // Throttle
    double throttle_cmd_;
    ros::Time throttle_stamp_;

    // Gear
    std_msgs::UInt8 gear_state_;
};

GZ_REGISTER_MODEL_PLUGIN(SequoiaInterfacePlugin)

}

#endif // AUDIBOTINTERFACEPLUGIN_H
