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
#include <dynamic_reconfigure/server.h>
#include <sequoia_lane_detection/LaneDetectionConfig.h>
#include <visualization_msgs/Marker.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/PointCloud.h>

namespace sequoia_lane_detection
{

class LaneDetection
{
public:
  LaneDetection(ros::NodeHandle n, ros::NodeHandle pn);

private:
  void reconfig(LaneDetectionConfig& config, uint32_t level);
  void recvImage(const sensor_msgs::ImageConstPtr& msg);
  void recvCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);

  void getBboxes(const cv::Mat& bin_img, cv::Mat& label_viz_img);
  void fitSegments(cv::Mat& bin_img, std::vector<Eigen::VectorXd>& fit_params);
  int sampleCurve(const Eigen::VectorXd& params, int y);

  std::vector<cv::Vec2f> detectStopLine(const cv::Mat& bin_img);

  geometry_msgs::Point32 projectPoint(const image_geometry::PinholeCameraModel& model,
                                      const tf::StampedTransform& transform,
                                      const tf::Vector3& cam_los_vect,
                                      const cv::Point2d& p);

  tf::TransformListener listener_;

  ros::Subscriber sub_image_;
  ros::Subscriber sub_cam_info_;
  ros::Publisher pub_line_obstacles_;
  ros::Publisher pub_viz_obstacles_;
  ros::Publisher pub_line_cloud_;

  dynamic_reconfigure::Server<LaneDetectionConfig> srv_;
  LaneDetectionConfig cfg_;

  sensor_msgs::CameraInfo camera_info_;
  std::vector<cv::Rect> bboxes;

};

}

#endif // LANEDETECTION_H
