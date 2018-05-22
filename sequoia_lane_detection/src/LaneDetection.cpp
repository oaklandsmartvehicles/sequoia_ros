#include "LaneDetection.h"

namespace sequoia_lane_detection {

LaneDetection::LaneDetection(ros::NodeHandle n, ros::NodeHandle pn)
{
  sub_cam_info_ = n.subscribe("camera_info", 1, &LaneDetection::recvCameraInfo, this);
  sub_image_ = n.subscribe("image_rect_color", 1, &LaneDetection::recvImage, this);
  pub_line_obstacles_ = n.advertise<costmap_converter::ObstacleArrayMsg>("line_obstacles", 1);
}

void LaneDetection::recvImage(const sensor_msgs::ImageConstPtr& msg)
{
  // Convert ROS image message into a cv::Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat raw_img = cv_ptr->image;

  // Image processing goes here. Need vector of Hough line segments as output
  std::vector<cv::Vec4i> line_segments;
//   cv::HoughLinesP(dilate_img, line_segments, cfg_.hough_rho_res, cfg_.hough_theta_res, cfg_.hough_threshold, cfg_.hough_min_length, cfg_.hough_max_gap);


  // Project Hough transform lines from camera into vehicle frame and publish as obstacles
  tf::StampedTransform transform;
  try{
    listener_.lookupTransform("base_footprint", msg->header.frame_id, msg->header.stamp, transform);
  } catch (tf::TransformException& ex){
    ROS_WARN_THROTTLE(1.0, "%s", ex.what());
    return;
  }

  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info_);
  tf::Vector3 cam_los_vect = transform(tf::Vector3(0, 0, 0));

  costmap_converter::ObstacleArrayMsg obstacle_msg;
  obstacle_msg.header.frame_id = "base_footprint";
  obstacle_msg.header.stamp = msg->header.stamp;

  for (size_t i=0; i<line_segments.size(); i++) {
    cv::Point2d p1(line_segments[i][0], line_segments[i][1]);
    cv::Point2d p2(line_segments[i][2], line_segments[i][3]);

    costmap_converter::ObstacleMsg line_obstacle;
    line_obstacle.header = obstacle_msg.header;
    line_obstacle.id = i;
    line_obstacle.orientation.w = 1.0;
    line_obstacle.polygon.points.push_back(projectPoint(model, transform, cam_los_vect, p1));
    line_obstacle.polygon.points.push_back(projectPoint(model, transform, cam_los_vect, p2));
    obstacle_msg.obstacles.push_back(line_obstacle);
  }

  pub_line_obstacles_.publish(obstacle_msg);
}

geometry_msgs::Point32 LaneDetection::projectPoint(const image_geometry::PinholeCameraModel& model,
                                                   const tf::StampedTransform& transform,
                                                   const tf::Vector3& cam_los_vect,
                                                   const cv::Point2d& p)
{
  cv::Point3d p3d = model.projectPixelTo3dRay(p);
  tf::Vector3 v1 = transform(tf::Vector3(p3d.x, p3d.y, p3d.z));
  double d = -cam_los_vect.z() / (v1.z() - cam_los_vect.z());
  tf::Vector3 v_robot(d * (v1.x() - cam_los_vect.x()) + cam_los_vect.x(), d * (v1.y() - cam_los_vect.y()) + cam_los_vect.y(), 0);

  geometry_msgs::Point32 point;
  point.x = v_robot.x();
  point.y = v_robot.y();
  point.z = v_robot.z();
  return point;
}

void LaneDetection::recvCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg)
{
  camera_info_ = *msg;
}

}
