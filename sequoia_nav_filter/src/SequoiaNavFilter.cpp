#include "SequoiaNavFilter.h"

using namespace Eigen;

namespace sequoia_nav_filter {

SequoiaNavFilter::SequoiaNavFilter(ros::NodeHandle n, ros::NodeHandle pn)
{
  pn.param("parent_frame", parent_frame_, std::string("map"));
  pn.param("child_frame", child_frame_, std::string("base_footprint"));
  pn.param("antenna_frame", antenna_frame_, std::string("gps_antenna"));
  pn.param("initial_heading", initial_heading_, 0.0);

  sub_rtk = new message_filters::Subscriber<nav_msgs::Odometry>(n, "gps/rtkfix", 1);
  sub_vehicle_data = new message_filters::Subscriber<geometry_msgs::TwistStamped>(n, "/vehicle/twist", 1);
  sync_ekf_input_data = new message_filters::Synchronizer<EkfDataPolicy>(EkfDataPolicy(10), *sub_vehicle_data, *sub_rtk);
  sync_ekf_input_data->registerCallback(boost::bind(&SequoiaNavFilter::recvData, this, _1, _2));
  pub_heading = n.advertise<std_msgs::Float64>("ekf_heading", 1);
  pub_odom = n.advertise<nav_msgs::Odometry>("ekf_odom", 1);
  srv_.setCallback(boost::bind(&SequoiaNavFilter::reconfig, this, _1, _2));

  last_stamp_ = ros::Time(0);
  got_gps_ant_offset_ = false;
  stationary_ = true;
  tf_lookup_try_count_ = 0;

  upsample_timer = n.createTimer(ros::Duration(0.02), &SequoiaNavFilter::timerCb, this);
}

void SequoiaNavFilter::timerCb(const ros::TimerEvent& event)
{
  if (last_stamp_ == ros::Time(0) || stationary_) {
    return;
  }

  double sample_time = (event.current_real - last_stamp_).toSec();
  last_stamp_ = event.current_real;

  EkfPrediction(sample_time, X, P);
  publishOutputs(event.current_real);
}

void SequoiaNavFilter::EkfPrediction(double sample_time, Matrix<double, 5, 1>& predicted_state, Matrix<double, 5, 5>& predicted_cov)
{
  // Prediction step
  Matrix<double, 5, 5> A;
  A.row(0) << 1, 0, -sample_time * X(SPEED) * sin(X(HEADING)), sample_time * cos(X(HEADING)), 0;
  A.row(1) << 0, 1, sample_time * X(SPEED) * cos(X(HEADING)), sample_time * sin(X(HEADING)), 0;
  A.row(2) << 0, 0, 1, 0, sample_time;
  A.row(3) << 0, 0, 0, 1, 0;
  A.row(4) << 0, 0, 0, 0, 1;

  predicted_state(0) = X(0) + sample_time * X(SPEED) * cos(X(HEADING));
  predicted_state(1) = X(1) + sample_time * X(SPEED) * sin(X(HEADING));
  predicted_state(2) = X(2) + sample_time * X(YAW_RATE);
  predicted_state(3) = X(3);
  predicted_state(4) = X(4);

  predicted_cov = A * P * A.transpose() + Q;
}

void SequoiaNavFilter::recvData(const geometry_msgs::TwistStampedConstPtr& vehicle_data,
                                const nav_msgs::OdometryConstPtr& rtk_data)
{
  // Look up TF transform to get GPS antenna offset
  if (!got_gps_ant_offset_ && tf_lookup_try_count_ < 10) {
    tf::StampedTransform transform;
    try {
      listener.lookupTransform("base_footprint", "gps_antenna", rtk_data->header.stamp, transform);
      gps_ant_offset_ = transform.getOrigin();
      got_gps_ant_offset_ = true;
    } catch (tf::TransformException& ex) {
      tf_lookup_try_count_++;
    }
  }

  // Initialize filter with first sample
  if (last_stamp_ == ros::Time(0)) {
    X(POS_X) = rtk_data->pose.pose.position.x;
    X(POS_Y) = rtk_data->pose.pose.position.y;
    X(HEADING) = initial_heading_;
    X(SPEED) = vehicle_data->twist.linear.x;
    X(YAW_RATE) = vehicle_data->twist.angular.z;

    P.setIdentity();

    last_stamp_ = rtk_data->header.stamp;
    return;
  }

  // Lock filter updates if not moving
  if (vehicle_data->twist.linear.x < 0.01) {
    stationary_ = true;
    publishOutputs(rtk_data->header.stamp);
    return;
  } else {
    stationary_ = false;
  }

  // Compute sample time from delta since last measurement input
  double sample_time = (rtk_data->header.stamp - last_stamp_).toSec();
  last_stamp_ = rtk_data->header.stamp;

  Matrix<double, 5, 5> predicted_cov;
  Matrix<double, 5, 1> predicted_state;
  EkfPrediction(sample_time, predicted_state, predicted_cov);

  // Update Step
  VectorXd measurements;
  VectorXd expected_measurements;
  MatrixXd C;
  MatrixXd R;

  double c13 = -gps_ant_offset_.x() * sin(X(HEADING)) - gps_ant_offset_.y() * cos(X(HEADING));
  double c23 = gps_ant_offset_.x() * cos(X(HEADING)) - gps_ant_offset_.y() * sin(X(HEADING));
  if (vehicle_data->twist.linear.x < cfg_.min_gps_vel) {
    C.setZero(4, 5);
    measurements.setZero(4);
    expected_measurements.setZero(4);
    R.setZero(4, 4);

    C.row(0) << 1, 0, c13, 0, 0;
    C.row(1) << 0, 1, c23, 0, 0;
    C.row(2) << 0, 0, 0, 1, 0;
    C.row(3) << 0, 0, 0, 0, 1;

    measurements << rtk_data->pose.pose.position.x, rtk_data->pose.pose.position.y,
                    vehicle_data->twist.linear.x, vehicle_data->twist.angular.z;

    double x_meas = X(POS_X) + gps_ant_offset_.x() * cos(X(HEADING)) - gps_ant_offset_.y() * sin(X(HEADING));
    double y_meas = X(POS_Y) + gps_ant_offset_.x() * sin(X(HEADING)) + gps_ant_offset_.y() * cos(X(HEADING));
    expected_measurements << x_meas, y_meas, X(SPEED), X(YAW_RATE);

    R(0, 0) = cfg_.r_gps * cfg_.r_gps;
    R(1, 1) = cfg_.r_gps * cfg_.r_gps;
    R(2, 2) = cfg_.r_speed * cfg_.r_speed;
    R(3, 3) = cfg_.r_yaw_rate * cfg_.r_yaw_rate;

  } else {
    C.setZero(6, 5);
    measurements.setZero(6);
    expected_measurements.setZero(6);
    R.setZero(6, 6);

    C.row(0) << 1, 0, c13, 0, 0;
    C.row(1) << 0, 1, c23, 0, 0;
    C.row(2) << 0, 0, -X(SPEED) * sin(X(HEADING)), cos(X(HEADING)), 0;
    C.row(3) << 0, 0, X(SPEED) * cos(X(HEADING)), sin(X(HEADING)), 0;
    C.row(4) << 0, 0, 0, 1, 0;
    C.row(5) << 0, 0, 0, 0, 1;

    measurements << rtk_data->pose.pose.position.x, rtk_data->pose.pose.position.y,
                    rtk_data->twist.twist.linear.x, rtk_data->twist.twist.linear.y,
                    vehicle_data->twist.linear.x, vehicle_data->twist.angular.z;

    double x_meas = X(POS_X) + gps_ant_offset_.x() * cos(X(HEADING)) - gps_ant_offset_.y() * sin(X(HEADING));
    double y_meas = X(POS_Y) + gps_ant_offset_.x() * sin(X(HEADING)) + gps_ant_offset_.y() * cos(X(HEADING));
    expected_measurements << x_meas, y_meas, X(SPEED) * cos(X(HEADING)), X(SPEED) * sin(X(HEADING)), X(SPEED), X(YAW_RATE);

    R(0, 0) = cfg_.r_gps * cfg_.r_gps;
    R(1, 1) = cfg_.r_gps * cfg_.r_gps;
    R(2, 2) = cfg_.r_gps_vel * cfg_.r_gps_vel;
    R(3, 3) = cfg_.r_gps_vel* cfg_.r_gps_vel;
    R(4, 4) = cfg_.r_speed * cfg_.r_speed;
    R(5, 5) = cfg_.r_yaw_rate * cfg_.r_yaw_rate;
  }

  MatrixXd S = C * predicted_cov * C.transpose() + R;
  MatrixXd K = predicted_cov * C.transpose() * S.inverse();

  MatrixXd error = measurements - expected_measurements;

  X = predicted_state + K * error;
  P = (MatrixXd::Identity(5, 5) - K*C) * predicted_cov;
}

void SequoiaNavFilter::publishOutputs(const ros::Time& stamp)
{
  // TF frame
  tf::StampedTransform ekf_transform;
  ekf_transform.stamp_ = stamp;
  ekf_transform.frame_id_ = parent_frame_;
  ekf_transform.child_frame_id_ = child_frame_;
  ekf_transform.setRotation(tf::createQuaternionFromYaw(X(HEADING)));
  ekf_transform.setOrigin(tf::Vector3(X(POS_X), X(POS_Y), 0));
  br.sendTransform(ekf_transform);

  // Odometry
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = parent_frame_;
  odom_msg.child_frame_id = child_frame_;
  odom_msg.pose.pose.position.x = X(POS_X);
  odom_msg.pose.pose.position.y = X(POS_Y);
  odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(X(HEADING));
  pub_odom.publish(odom_msg);

  // Heading`
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

  cfg_ = config;
//   R.setZero();
//   R(0, 0) = config.r_gps * config.r_gps;
//   R(1, 1) = config.r_gps * config.r_gps;
//   R(2, 2) = config.r_gps_vel * config.r_gps_vel;
//   R(3, 3) = config.r_gps_vel* config.r_gps_vel;
//   R(4, 4) = config.r_speed * config.r_speed;
//   R(5, 5) = config.r_yaw_rate * config.r_yaw_rate;
}

}
