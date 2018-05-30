#include "SequoiaNavFilter.h"

using namespace Eigen;

namespace sequoia_nav_filter {

SequoiaNavFilter::SequoiaNavFilter(ros::NodeHandle n, ros::NodeHandle pn)
{
  pn.param("parent_frame", parent_frame_, std::string("map"));
  pn.param("child_frame", child_frame_, std::string("base_footprint"));
  pn.param("initial_heading", initial_heading_, 0.0);

  sub_rtk = new message_filters::Subscriber<nav_msgs::Odometry>(n, "gps/rtkfixed", 1);
  sync_ekf_input_data = new message_filters::Synchronizer<EkfDataPolicy>(EkfDataPolicy(10), *sub_vehicle_data, *sub_rtk);
  sync_ekf_input_data->registerCallback(boost::bind(&SequoiaNavFilter::recvData, this, _1, _2));
  pub_heading = n.advertise<std_msgs::Float64>("ekf_heading", 1);
  srv_.setCallback(boost::bind(&SequoiaNavFilter::reconfig, this, _1, _2));

  last_data_stamp_ = ros::Time(0);
}

void SequoiaNavFilter::recvData(const geometry_msgs::TwistStampedConstPtr& vehicle_data,
                                const nav_msgs::OdometryConstPtr& rtk_data)
{
  // Initialize filter with first sample
  if (last_data_stamp_ == ros::Time(0)) {
    X(POS_X) = rtk_data->pose.pose.position.x;
    X(POS_Y) = rtk_data->pose.pose.position.y;
    X(HEADING) = initial_heading_;
    X(SPEED) = vehicle_data->twist.linear.x;
    X(YAW_RATE) = vehicle_data->twist.angular.z;

    P.setIdentity();

    last_data_stamp_ = rtk_data->header.stamp;
    return;
  }

  // Lock filter updates if not moving
  if (vehicle_data->twist.linear.x < 0.01) {
    publishOutputs(rtk_data->header.stamp);
    return;
  }

  // Compute sample time from delta since last measurement input
  double sample_time = (rtk_data->header.stamp - last_data_stamp_).toSec();
  last_data_stamp_ = rtk_data->header.stamp;

  // Prediction step
  Matrix<double, 5, 5> A;
  A.row(0) << 1, 0, -sample_time * X(SPEED) * sin(X(HEADING)), sample_time * cos(X(HEADING)), 0;
  A.row(1) << 0, 1, sample_time * X(SPEED) * cos(X(HEADING)), sample_time * sin(X(HEADING)), 0;
  A.row(2) << 0, 0, 1, 0, sample_time;
  A.row(3) << 0, 0, 0, 1, 0;
  A.row(4) << 0, 0, 0, 0, 1;

  Matrix<double, 5, 1> predicted_state;
  predicted_state(0) = X(0) + sample_time * X(SPEED) * cos(X(HEADING));
  predicted_state(1) = X(1) + sample_time * X(SPEED) * sin(X(HEADING));
  predicted_state(2) = X(2) + sample_time * X(YAW_RATE);
  predicted_state(3) = X(3);
  predicted_state(4) = X(4);

  Matrix<double, 5, 5> predicted_cov;
  predicted_cov = A * P * A.transpose() + Q;

  // Update Step
  Matrix<double, 6, 1> measurements;
  Matrix<double, 6, 1> expected_measurements;
  C.setZero();
  // Update C matrix here

  // Populate actual measurement vector here

  // Populate expected measurement vector here

  MatrixXd S = C * predicted_cov * C.transpose() + R;
  MatrixXd K = predicted_cov * C.transpose() * S.inverse();

  MatrixXd error = measurements - expected_measurements;

  X = predicted_state + K * error;
  P = (MatrixXd::Identity(5, 5) - K*C) * predicted_cov;

  publishOutputs(rtk_data->header.stamp);
}

void SequoiaNavFilter::publishOutputs(const ros::Time& stamp)
{
  tf::StampedTransform ekf_transform;
  ekf_transform.stamp_ = stamp;
  ekf_transform.frame_id_ = parent_frame_;
  ekf_transform.child_frame_id_ = child_frame_;
  ekf_transform.setRotation(tf::createQuaternionFromYaw(X(HEADING)));
  ekf_transform.setOrigin(tf::Vector3(X(POS_X), X(POS_Y), 0));
  br.sendTransform(ekf_transform);
  std_msgs::Float64 heading;
  heading.data = X(HEADING);
  pub_heading.publish(heading);
}

void SequoiaNavFilter::reconfig(EkfConfig& config, uint32_t level)
{
  Q.setZero();
  Q(0, 0) = config.q_pos * config.q_pos;
  Q(1, 1) = config.q_pos * config.q_pos;
  Q(2, 2) = config.q_heading * config.q_heading;
  Q(3, 3) = config.q_speed * config.q_speed;
  Q(4, 4) = config.q_yaw_rate * config.q_yaw_rate;

  R.setZero();
  R(0, 0) = config.r_gps * config.r_gps;
  R(1, 1) = config.r_gps * config.r_gps;
  R(2, 2) = config.r_gps_vel * config.r_gps_vel;
  R(3, 3) = config.r_gps_vel* config.r_gps_vel;
  R(4, 4) = config.r_speed * config.r_speed;
  R(5, 5) = config.r_yaw_rate * config.r_yaw_rate;
}

}
