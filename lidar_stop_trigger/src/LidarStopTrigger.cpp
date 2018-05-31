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
  // Search for closest point in a y window
  std::vector<double> possible_candidates;

  for (size_t i=0; i<msg->ranges.size(); i++) {
    if (msg->ranges[i] > msg->range_min) {
      double angle = msg->angle_min + i * msg->angle_increment;
      double x = msg->ranges[i] * cos(angle);
      double y = msg->ranges[i] * sin(angle);

      if (fabs(y) < 0.5 * cfg_.window_width) {
        possible_candidates.push_back(x);
      }
    }
  }

  double min_dist = INFINITY;
  for (size_t i=0; i<possible_candidates.size(); i++) {
    if (possible_candidates[i] < min_dist) {
      min_dist = possible_candidates[i];
    }
  }

  // Apply hysteresis to avoid chattering of stop signal,
  // and publish when trigger changes state
  if (!stop_trigger_msg_.data) {
    if (min_dist < cfg_.sensitve_range) {
      stop_trigger_msg_.data = true;
      pub_stop_trigger_.publish(stop_trigger_msg_);
    }
  } else {
    if (min_dist > (cfg_.sensitve_range + cfg_.range_hyst)) {
      stop_trigger_msg_.data = false;
      pub_stop_trigger_.publish(stop_trigger_msg_);
    }
  }
}

}
