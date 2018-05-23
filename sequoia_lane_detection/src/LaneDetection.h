#ifndef LANEDETECTION_H
#define LANEDETECTION_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <costmap_converter/ObstacleArrayMsg.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <image_geometry/pinhole_camera_model.h>

namespace sequoia_lane_detection
{

class LaneDetection
{
public:
  LaneDetection(ros::NodeHandle n, ros::NodeHandle pn);

private:
  void recvImage(const sensor_msgs::ImageConstPtr& msg);
  void recvCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);

  geometry_msgs::Point32 projectPoint(const image_geometry::PinholeCameraModel& model,
                                      const tf::StampedTransform& transform,
                                      const tf::Vector3& cam_los_vect,
                                      const cv::Point2d& p);

  tf::TransformListener listener_;

  ros::Subscriber sub_image_;
  ros::Subscriber sub_cam_info_;
  ros::Publisher pub_line_obstacles_;

  sensor_msgs::CameraInfo camera_info_;

};

}

#endif // LANEDETECTION_H
