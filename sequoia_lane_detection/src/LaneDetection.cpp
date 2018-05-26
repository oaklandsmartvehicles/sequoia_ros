#include "LaneDetection.h"

#define DEBUG 1

using namespace cv;

namespace sequoia_lane_detection
{

LaneDetection::LaneDetection(ros::NodeHandle n, ros::NodeHandle pn)
{
  sub_cam_info_ = n.subscribe("camera_info", 1, &LaneDetection::recvCameraInfo, this);
  sub_image_ = n.subscribe("image_raw", 1, &LaneDetection::recvImage, this);
  pub_line_obstacles_ = n.advertise<costmap_converter::ObstacleArrayMsg>("line_obstacles", 1);
  pub_viz_obstacles_ = n.advertise<visualization_msgs::Marker>("viz_line_obstacles", 1);
  pub_line_scan_ = n.advertise<sensor_msgs::LaserScan>("line_scan", 1);

  srv_.setCallback(boost::bind(&LaneDetection::reconfig, this, _1, _2));

#if DEBUG
  namedWindow("Output Image", CV_WINDOW_NORMAL);
  namedWindow("Binary", CV_WINDOW_NORMAL);
#endif
}

void LaneDetection::getBboxes(const Mat& bin_img, Mat& label_viz_img)
{
  Mat label_img;
  Mat stats;
  Mat centroids;
  int num_labels = connectedComponentsWithStats(bin_img, label_img, stats, centroids);

  bboxes.clear();
  for (int label = 1; label < num_labels; label++) {
    int x0 = stats.at<int>(label, CC_STAT_LEFT);
    int y0 = stats.at<int>(label, CC_STAT_TOP);
    int w = stats.at<int>(label, CC_STAT_WIDTH);
    int h = stats.at<int>(label, CC_STAT_HEIGHT);

    if (h > cfg_.min_seg_height) {
      bboxes.push_back(Rect(x0, y0, w, h));
    }
  }

#if DEBUG
  label_viz_img = Mat(bin_img.size(), CV_8UC3);

  std::vector<Vec3b> colors(num_labels);
  colors[0] = Vec3b(0, 0, 0);
  for (int label = 1; label < num_labels; label++) { //label  0 is the background
    switch (label % 4) {
      case 0:
        colors[label] = Vec3b(0, 255, 0);
        break;
      case 1:
        colors[label] = Vec3b(255, 0, 255);
        break;
      case 2:
        colors[label] = Vec3b(255, 255, 0);
        break;
      case 3:
        colors[label] = Vec3b(0, 255, 255);
        break;
    }
  }

  for (int r = 0; r < label_viz_img.rows; r++) {
    for (int c = 0; c < label_viz_img.cols; c++) {
      int label = label_img.at<int>(r, c);
      Vec3b& pixel = label_viz_img.at<Vec3b>(r, c);
      if (label == 0) {
        pixel = Vec3b(0, 0, 0);
      }
      pixel = colors[label];
    }
  }
#endif
}

void LaneDetection::fitSegments(Mat& bin_img, std::vector<Eigen::VectorXd>& fit_params)
{
  fit_params.resize(bboxes.size());
  for (size_t i = 0; i < bboxes.size(); i++) {
    // Run Canny edge detection to greatly cut down the number of points
    Mat canny_edge;
    Canny(bin_img(bboxes[i]), canny_edge, 2, 4);

    std::vector<int> x_samples;
    std::vector<int> y_samples;
    for (int xx = 0; xx < canny_edge.cols; xx++) {
      for (int yy = 0; yy < canny_edge.rows; yy++) {
        if (canny_edge.at<uint8_t>(Point(xx, yy)) == 255) {
          x_samples.push_back(xx);
          y_samples.push_back(yy);
        }
      }
    }

    Eigen::VectorXd x_samples_eig(x_samples.size());
    Eigen::MatrixXd regression_mat(y_samples.size(), 4);
    for (size_t j = 0; j < x_samples.size(); j++) {
      x_samples_eig(j) = (double)x_samples[j];
      regression_mat.row(j) << (double)(y_samples[j] * y_samples[j] * y_samples[j]), (double)(y_samples[j] * y_samples[j]), (double)y_samples[j], 1.0;
    }

    Eigen::MatrixXd temp = regression_mat.transpose() * regression_mat;
    Eigen::VectorXd params = temp.inverse() * regression_mat.transpose() * x_samples_eig;

    double error = 0;
    for (size_t j = 0; j < x_samples.size(); j++) {
      double x_est = params(0) * y_samples[j] * y_samples[j] * y_samples[j] + params(1) * y_samples[j] * y_samples[j] + params(2) * y_samples[j] + params(3);
      error += fabs(x_est - x_samples[j]);
    }
    error /= (double)x_samples.size();

    if (error < cfg_.fit_tolerance) {
      fit_params[i] = params;
    } else {
      fit_params[i] = Eigen::VectorXd();
    }

  }
}

std::vector<Vec2f> LaneDetection::detectStopLine(const Mat& bin_img)
{
  std::vector<Vec2f> output;
  for (size_t i=0; i<bboxes.size(); i++) {
    std::vector<Vec2f> hough_lines;
    Mat canny_edge;
    Canny(bin_img(bboxes[i]), canny_edge, 2, 4);
    cv::HoughLines(canny_edge, hough_lines, cfg_.hough_rho_res, cfg_.hough_theta_res, cfg_.hough_threshold, 0, 0,
                   M_PI/2 - cfg_.hough_horiz_tol, M_PI/2 + cfg_.hough_horiz_tol);

    Vec2f avg_line(0, 0);
    for (size_t j=0; j<hough_lines.size(); j++) {
      avg_line[0] += hough_lines[j][0];
      avg_line[1] += hough_lines[j][1];
    }
    avg_line[0] /= (float)hough_lines.size();
    avg_line[1] /= (float)hough_lines.size();

    output.push_back(avg_line);
  }

  return output;
}

int LaneDetection::sampleCurve(const Eigen::VectorXd& params, int y)
{
  return params(0) * y * y * y + params(1) * y * y + params(2) * y + params(3);
}

void LaneDetection::recvImage(const sensor_msgs::ImageConstPtr& msg)
{
  // Convert ROS image message into a Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  Mat raw_img = cv_ptr->image;
  cv::GaussianBlur(raw_img, raw_img, Size(cfg_.blur_kernel, cfg_.blur_kernel), 0, 0);

  Mat raw_hsv;
  cvtColor(raw_img, raw_hsv, CV_BGR2HSV);

  std::vector<Mat> split_img;
  split(raw_hsv, split_img);
  Mat val_img = split_img[2];
  Mat sat_img = split_img[1];

  Mat val_thres_img;
  threshold(val_img, val_thres_img, cfg_.val_thres, 255, CV_THRESH_BINARY);

  Mat sat_thres_img;
  threshold(sat_img, sat_thres_img, cfg_.sat_thres, 255, CV_THRESH_BINARY_INV);

  Mat bin_img;
  bitwise_and(val_thres_img, sat_thres_img, bin_img);

  erode(bin_img, bin_img, Mat::ones(cfg_.erode_size, cfg_.erode_size, CV_8U));

  Mat labeled_image;
  getBboxes(bin_img, labeled_image);

  std::vector<Vec2f> stop_line = detectStopLine(bin_img);

  std::vector<Eigen::VectorXd> fit_params;
  fitSegments(bin_img, fit_params);

  std::vector<std::vector<cv::Point> > sampled_points;
  for (size_t i = 0; i < fit_params.size(); i++) {
    if (fit_params[i].rows() == 0) {
      continue;
    }

    std::vector<cv::Point> segment_points;
    cv::Point new_point;
    for (int y = 0; y < bboxes[i].height; y += cfg_.reconstruct_pix) {
      new_point.x = sampleCurve(fit_params[i], y) + bboxes[i].x;
      new_point.y = y + bboxes[i].y;
      segment_points.push_back(new_point);
    }
    new_point.x = sampleCurve(fit_params[i], bboxes[i].height) + bboxes[i].x;
    new_point.y = bboxes[i].height + bboxes[i].y;
    segment_points.push_back(new_point);
    sampled_points.push_back(segment_points);
  }

#if DEBUG
  imshow("Binary", labeled_image);
  waitKey(1);

  for (size_t i = 0; i < bboxes.size(); i++) {
    cv::rectangle(raw_img, bboxes[i], cv::Scalar(0, 255, 0));
  }

  for (size_t i = 0; i < sampled_points.size(); i++) {
    for (size_t j = 1; j < sampled_points[i].size(); j++) {
      cv::line(raw_img, sampled_points[i][j - 1], sampled_points[i][j], cv::Scalar(0, 0, 255));
    }
  }

  for (size_t i=0; i<stop_line.size(); i++) {
    float rho = stop_line[i][0], theta = stop_line[i][1];
    Point p1, p2;
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    p1.x = cvRound(x0 + 1000*(-b)) + bboxes[i].x;
    p1.y = cvRound(y0 + 1000*(a)) + bboxes[i].y;
    p2.x = cvRound(x0 - 1000*(-b)) + bboxes[i].x;
    p2.y = cvRound(y0 - 1000*(a)) + bboxes[i].y;
    line(raw_img, p1, p2, cv::Scalar(255, 0, 0));
  }

  imshow("Output Image", raw_img);
  waitKey(1);
#endif

  // Project output lines from camera into vehicle frame and publish as obstacles
  tf::StampedTransform transform;
  try {
    listener_.lookupTransform("base_footprint", msg->header.frame_id, msg->header.stamp, transform);
  } catch (tf::TransformException& ex) {
    ROS_WARN_THROTTLE(1.0, "%s", ex.what());
    return;
  }

  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info_);
  tf::Vector3 cam_los_vect = transform(tf::Vector3(0, 0, 0));

  costmap_converter::ObstacleArrayMsg obstacle_msg;
  obstacle_msg.header.frame_id = "base_footprint";
  obstacle_msg.header.stamp = msg->header.stamp;

  visualization_msgs::Marker viz_marker;
  viz_marker.header.frame_id = "base_footprint";
  viz_marker.header.stamp = msg->header.stamp;
  viz_marker.action = visualization_msgs::Marker::ADD;
  viz_marker.type = visualization_msgs::Marker::LINE_LIST;
  viz_marker.color.a = 1.0;
  viz_marker.color.r = 1.0;
  viz_marker.color.g = 1.0;
  viz_marker.color.b = 1.0;
  viz_marker.scale.x = 0.1;
  viz_marker.pose.orientation.w = 1;

  std::vector<geometry_msgs::Point32> line_points;

  for (size_t i = 0; i < sampled_points.size(); i++) {
    for (size_t j = 1; j < sampled_points[i].size(); j++) {

      costmap_converter::ObstacleMsg line_obstacle;
      line_obstacle.header = obstacle_msg.header;
      line_obstacle.id = i * (sampled_points[i].size() - 1) + j;
      line_obstacle.orientation.w = 1.0;
      geometry_msgs::Point32 p1 = projectPoint(model, transform, cam_los_vect, sampled_points[i][j - 1]);
      geometry_msgs::Point32 p2 = projectPoint(model, transform, cam_los_vect, sampled_points[i][j]);
      line_obstacle.polygon.points.push_back(p1);
      line_obstacle.polygon.points.push_back(p2);
      obstacle_msg.obstacles.push_back(line_obstacle);
      line_points.push_back(p1);

      geometry_msgs::Point temp;
      temp.x = p1.x;
      temp.y = p1.y;
      temp.z = p1.z;
      viz_marker.points.push_back(temp);
      temp.x = p2.x;
      temp.y = p2.y;
      temp.z = p2.z;
      viz_marker.points.push_back(temp);
    }
  }

  sensor_msgs::LaserScan scan_msg;
  scan_msg.header.frame_id = "base_footprint";
  scan_msg.header.stamp = msg->header.stamp;
  scan_msg.angle_min = -M_PI / 2;
  scan_msg.angle_max = M_PI / 2;
  scan_msg.angle_increment = 0.03;
  scan_msg.range_min = 0.0;
  scan_msg.range_max = 30.0;
  scan_msg.ranges.resize((int)((scan_msg.angle_max - scan_msg.angle_min ) / scan_msg.angle_increment), INFINITY);
  for (size_t i=0; i<line_points.size(); i++) {
    double range2 = line_points[i].x*line_points[i].x + line_points[i].y*line_points[i].y;
    double angle = atan2(line_points[i].y, line_points[i].x);
    int angle_bin = (int)((angle - scan_msg.angle_min) / scan_msg.angle_increment);

    if ((angle_bin < 0) || (angle_bin > (scan_msg.ranges.size()-1))) {
      continue;
    }
    if (range2 < scan_msg.ranges[angle_bin] * scan_msg.ranges[angle_bin]) {
      scan_msg.ranges[angle_bin] = sqrt(range2);
    }
  }

  pub_line_scan_.publish(scan_msg);
  pub_viz_obstacles_.publish(viz_marker);
  pub_line_obstacles_.publish(obstacle_msg);

}

void LaneDetection::reconfig(LaneDetectionConfig& config, uint32_t level)
{
  if (!(config.erode_size % 2)) {
    config.erode_size--;
  }

  if (!(config.blur_kernel % 2)) {
    config.blur_kernel--;
  }

  cfg_ = config;
}

geometry_msgs::Point32 LaneDetection::projectPoint(const image_geometry::PinholeCameraModel& model,
    const tf::StampedTransform& transform,
    const tf::Vector3& cam_los_vect,
    const Point2d& p)
{
  Point3d p3d = model.projectPixelTo3dRay(p);
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
