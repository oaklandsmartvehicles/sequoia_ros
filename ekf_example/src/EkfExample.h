// Include guard to prevent multiple declarations
#ifndef EKFEXAMPLE_H
#define EKFEXAMPLE_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include "gps_conv.h"
#include <dynamic_reconfigure/server.h>
#include <ekf_example/EkfExampleConfig.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>



#include <eigen3/Eigen/Dense>

// Namespace matches ROS package name
namespace ekf_example{

class EkfExample
{
public:
  EkfExample(ros::NodeHandle n, ros::NodeHandle pn);

private:
  void reconfig(EkfExampleConfig& config, uint32_t level);
  void timerCallback(const ros::TimerEvent& event);
  void recvTwist(const geometry_msgs::TwistStampedConstPtr& msg);
  void recvGear(const std_msgs::UInt8::ConstPtr& msg);
  void recvFix(const sensor_msgs::NavSatFixConstPtr& msg);

  ros::Subscriber sub_twist;
  ros::Subscriber sub_fix;
  ros::Subscriber sub_gear;
  ros::Publisher pub_heading_;
  ros::Timer timer;

  dynamic_reconfigure::Server<EkfExampleConfig> srv_;

  geometry_msgs::Twist twist_data;
  tf::Vector3 position_data;
  UTMCoords ref_coords;
  tf::TransformBroadcaster broadcaster;
  double sample_time;
  std_msgs::UInt8 gear_data;


  Eigen::Matrix<double, 5, 5> Q;

  Eigen::Matrix<double, 4, 4> R_all;
  Eigen::Matrix<double, 2, 2> R_gps;
  Eigen::Matrix<double, 2, 2> R_twist;

  Eigen::Matrix<double, 5, 1> X;
  Eigen::Matrix<double, 5, 5> P;

  bool gps_available;
  bool twist_available;
  int latch_gps;
  int latch_heading;
  int gear_state_sign;
  
};

}

#endif // EKFEXAMPE_H

