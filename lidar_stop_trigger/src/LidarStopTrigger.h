#ifndef LIDARSTOPTRIGGER_H
#define LIDARSTOPTRIGGER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>

#include <dynamic_reconfigure/server.h>
#include <lidar_stop_trigger/LidarStopTriggerConfig.h>

namespace lidar_stop_trigger
{

class LidarStopTrigger
{
public:
  LidarStopTrigger(ros::NodeHandle n, ros::NodeHandle pn);
private:
  void reconfig(LidarStopTriggerConfig& config, uint32_t level);
  void recvLidarScan(const sensor_msgs::LaserScanConstPtr& msg);

  ros::Subscriber sub_laser_;
  ros::Publisher pub_stop_trigger_;

  dynamic_reconfigure::Server<LidarStopTriggerConfig> srv_;
  LidarStopTriggerConfig cfg_;

  std_msgs::Bool stop_trigger_msg_;

};

}

#endif // LIDARSTOPTRIGGER_H
