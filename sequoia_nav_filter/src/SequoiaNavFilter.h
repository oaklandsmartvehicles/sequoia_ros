#ifndef SEQUOIANAVFILTER_H
#define SEQUOIANAVFILTER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen3/Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>
#include <sequoia_nav_filter/EkfConfig.h>

namespace sequoia_nav_filter
{

  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, nav_msgs::Odometry> EkfDataPolicy;

  enum {POS_X=0, POS_Y, HEADING, SPEED, YAW_RATE, NUM_STATES};

class SequoiaNavFilter
{
public:
  SequoiaNavFilter(ros::NodeHandle n, ros::NodeHandle pn);

private:
  void reconfig(EkfConfig& config, uint32_t level);
  void recvData(const geometry_msgs::TwistStampedConstPtr& vehicle_data, const nav_msgs::OdometryConstPtr& rtk_data);
  void timerCb(const ros::TimerEvent& event);
  void publishOutputs(const ros::Time& stamp);
  void EkfPrediction(double sample_time, Eigen::Matrix<double, 5, 1>& predicted_state, Eigen::Matrix<double, 5, 5>& predicted_cov);

  message_filters::Subscriber<nav_msgs::Odometry>* sub_rtk;
  message_filters::Subscriber<geometry_msgs::TwistStamped>* sub_vehicle_data;
  message_filters::Synchronizer<EkfDataPolicy>* sync_ekf_input_data;
  ros::Publisher pub_heading;
  ros::Publisher pub_odom;
  ros::Timer upsample_timer;
  tf::TransformBroadcaster br;
  tf::TransformListener listener;
  dynamic_reconfigure::Server<EkfConfig> srv_;

  Eigen::Matrix<double, 5, 5> Q;
  Eigen::Matrix<double, 6, 6> R;
  Eigen::Matrix<double, 5, 1> X;
  Eigen::Matrix<double, 5, 5> P;
  Eigen::Matrix<double, 6, 5> C;

  ros::Time last_stamp_;
  tf::Vector3 gps_ant_offset_;
  std::string child_frame_;
  std::string parent_frame_;
  std::string antenna_frame_;
  double initial_heading_;
  bool got_gps_ant_offset_;
  bool stationary_;
  int tf_lookup_try_count_;


};

}

#endif // SEQUOIANAVFILTER_H
