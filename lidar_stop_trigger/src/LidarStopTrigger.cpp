#include "LidarStopTrigger.h"

namespace lidar_stop_trigger {

LidarStopTrigger::LidarStopTrigger(ros::NodeHandle n, ros::NodeHandle pn)
{
  srv_.setCallback(boost::bind(&LidarStopTrigger::reconfig, this, _1, _2));

  // Advertise boolean stop trigger signal as a latched topic
  pub_stop_trigger_ = n.advertise<std_msgs::Bool>("stop_trigger", 1, true);

  // Subscribe to raw laser scan to detect objects close to vehicle
  sub_laser_ = n.subscribe("scan", 1, &LidarStopTrigger::recvLidarScan, this);

  // Initialize stop trigger signal to false
  stop_trigger_msg_.data = false;
  pub_stop_trigger_.publish(stop_trigger_msg_);
}

// Update parameters from reconfigure GUI
void LidarStopTrigger::reconfig(LidarStopTriggerConfig& config, uint32_t level)
{
  cfg_ = config;
}

void LidarStopTrigger::recvLidarScan(const sensor_msgs::LaserScanConstPtr& msg)
{
  // Compute min and max indices of desired LIDAR scan ranges
  // based on configuration parameters
  int center_idx = msg->ranges.size() / 2;
  int half_angle_idx_range = (int)(0.5 * cfg_.angle_window / msg->angle_increment);
  int min_idx = center_idx - half_angle_idx_range;
  int max_idx = center_idx + half_angle_idx_range;

  // Find minimum range value within the min and max indices
  float min_range = *std::min_element(msg->ranges.begin() + min_idx, msg->ranges.begin() + max_idx);

  // Apply hysteresis to avoid chattering of stop signal,
  // and publish when trigger changes state
  if (!stop_trigger_msg_.data) {
    if (min_range < cfg_.sensitve_range) {
      stop_trigger_msg_.data = true;
      pub_stop_trigger_.publish(stop_trigger_msg_);
    }
  } else {
    if (min_range > (cfg_.sensitve_range + cfg_.range_hyst)) {
      stop_trigger_msg_.data = false;
      pub_stop_trigger_.publish(stop_trigger_msg_);
    }
  }
}

}
