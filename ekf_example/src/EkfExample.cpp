#include "EkfExample.h"

using namespace Eigen;

namespace ekf_example {

EkfExample::EkfExample(ros::NodeHandle n, ros::NodeHandle pn)
{
  // Set reference coordinates
  LatLon ref_point(42.6794081167, -83.1956015483, 5.0);
  ref_coords = UTMCoords(ref_point);

  // Base sampling time of the filter
  sample_time = 0.02;

  // Subscribe to input data, advertise path, and set up main filter timer
  sub_fix = n.subscribe("fix", 1, &EkfExample::recvFix, this);
  sub_twist = n.subscribe("vehicle/twist", 1, &EkfExample::recvTwist, this);
  timer = n.createTimer(ros::Duration(sample_time), &EkfExample::timerCallback, this);

  // Set up dynamic reconfigure server
  srv_.setCallback(boost::bind(&EkfExample::reconfig, this, _1, _2));
  
  X.setZero();
  P.setIdentity();
  
  twist_available = false;
  gps_available = false;
}

void EkfExample::recvTwist(const geometry_msgs::TwistStampedConstPtr& msg)
{
  twist_data = msg->twist;
  twist_available = true;
}

void EkfExample::recvFix(const sensor_msgs::NavSatFixConstPtr& msg)
{
  position_data = UTMCoords(*msg) - ref_coords;
  gps_available = true;
}

void EkfExample::timerCallback(const ros::TimerEvent& event)
{
  double heading_est = X(2);
  double speed_est = X(3);
  double yaw_rate_est = X(4);
  
  // Prediction step
  Matrix<double, 5, 5> A;
  A.row(0) << 1, 0, -sample_time * speed_est * sin(heading_est), sample_time * cos(heading_est), 0;
  A.row(1) << 0, 1, sample_time * speed_est * cos(heading_est), sample_time * sin(heading_est), 0;
  A.row(2) << 0, 0, 1, 0, sample_time;
  A.row(3) << 0, 0, 0, 1, 0;
  A.row(4) << 0, 0, 0, 0, 1;
  
  Matrix<double, 5, 1> predicted_state;
  predicted_state(0) = X(0) + sample_time * speed_est * cos(heading_est);
  predicted_state(1) = X(1) + sample_time * speed_est * sin(heading_est);
  predicted_state(2) = X(2) + sample_time * yaw_rate_est;
  predicted_state(3) = X(3);
  predicted_state(4) = X(4);
  
  Matrix<double, 5, 5> predicted_cov;
  predicted_cov = A * P * A.transpose() + Q;
  
  // Update Step
  MatrixXd R;
  MatrixXd C;
  MatrixXd measurements;
  MatrixXd expected_measurements;
  if (gps_available && twist_available) {
    R = R_all;
    C = MatrixXd(4, 5);
    C.row(0) << 1, 0, 0, 0, 0;
    C.row(1) << 0, 1, 0, 0, 0;
    C.row(2) << 0, 0, 0, 1, 0;
    C.row(3) << 0, 0, 0, 0, 1;
    
    measurements = MatrixXd(4, 1);
    measurements << position_data.x(), position_data.y(), twist_data.linear.x, twist_data.angular.z;
    
    expected_measurements = MatrixXd(4, 1);
    expected_measurements << predicted_state(0), predicted_state(1), predicted_state(3), predicted_state(4);
    
  } else if (gps_available) { 
    R = R_gps;
    C = MatrixXd(2, 5);
    C.row(0) << 1, 0, 0, 0, 0;
    C.row(1) << 0, 1, 0, 0, 0;
    
    measurements = MatrixXd(2, 1);
    measurements << position_data.x(), position_data.y();
    
    expected_measurements = MatrixXd(2, 1);
    expected_measurements << predicted_state(0), predicted_state(1);
    
  } else if (twist_available) {
    R = R_twist;
    C = MatrixXd(2, 5);
    C.row(0) << 0, 0, 0, 1, 0;
    C.row(1) << 0, 0, 0, 0, 1;
    
    measurements = MatrixXd(2, 1);
    measurements << twist_data.linear.x, twist_data.angular.z;
    
    expected_measurements = MatrixXd(2, 1);
    expected_measurements << predicted_state(3), predicted_state(4);
    
  }
  
  if (gps_available || twist_available) {
    MatrixXd S = C * predicted_cov * C.transpose() + R;
    MatrixXd K = predicted_cov * C.transpose() * S.inverse();
    
    MatrixXd error = measurements - expected_measurements;
    
    X = predicted_state + K * error;
    P = (MatrixXd::Identity(5, 5) - K*C) * predicted_cov;
  } else {
    X = predicted_state;
    P = predicted_cov;
  }
  
  gps_available = false;
  twist_available = false;
  
  // Update TF transform with Kalman filter output
  tf::StampedTransform ekf_transform;
  ekf_transform.stamp_ = event.current_real;
  ekf_transform.frame_id_ = "map";
  ekf_transform.child_frame_id_ = "base_footprint";
  ekf_transform.setRotation(tf::createQuaternionFromYaw(X(2)));
  ekf_transform.setOrigin(tf::Vector3(X(0), X(1), 0));
  broadcaster.sendTransform(ekf_transform);
  
  
}

void EkfExample::reconfig(EkfExampleConfig& config, uint32_t level)
{
  Q.setZero();
  Q(0, 0) = config.q_pos * config.q_pos;
  Q(1, 1) = config.q_pos * config.q_pos;
  Q(2, 2) = config.q_heading * config.q_heading;
  Q(3, 3) = config.q_speed * config.q_speed;
  Q(4, 4) = config.q_yaw_rate * config.q_yaw_rate;
  
  R_all.setZero();
  R_all(0, 0) = config.r_gps * config.r_gps;
  R_all(1, 1) = config.r_gps * config.r_gps;
  R_all(2, 2) = config.r_speed * config.r_speed;
  R_all(3, 3) = config.r_yaw_rate * config.r_yaw_rate;
  
  R_gps.setZero();
  R_gps(0, 0) = config.r_gps * config.r_gps;
  R_gps(1, 1) = config.r_gps * config.r_gps;
  
  R_twist.setZero();
  R_twist(0, 0) = config.r_speed * config.r_speed;
  R_twist(1, 1) = config.r_yaw_rate * config.r_yaw_rate;
}

}
