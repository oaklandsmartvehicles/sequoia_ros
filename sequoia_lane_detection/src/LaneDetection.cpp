#include "LaneDetection.h"

#define DEBUG 1

using namespace cv;

namespace sequoia_lane_detection
{

LaneDetection::LaneDetection(ros::NodeHandle n, ros::NodeHandle pn)
{
  sub_cam_info_ = n.subscribe("camera_info", 1, &LaneDetection::recvCameraInfo, this);
  sub_image_ = n.subscribe("image_raw", 1, &LaneDetection::recvImage, this);
  pub_line_visualization_ = n.advertise<visualization_msgs::Marker>("viz_line_obstacles", 1);
  pub_solid_line_cloud_ = n.advertise<sensor_msgs::PointCloud>("solid_line_cloud", 1);
  pub_dashed_line_cloud_ = n.advertise<sensor_msgs::PointCloud>("dashed_line_cloud", 1);
  pub_stop_line_dist_ = n.advertise<std_msgs::Float64>("stop_line_dist", 1);

  srv_.setCallback(boost::bind(&LaneDetection::reconfig, this, _1, _2));

  downsample_count = 0;

#if DEBUG
  namedWindow("Output Image", CV_WINDOW_NORMAL);
  namedWindow("Binary", CV_WINDOW_NORMAL);
#endif
}

// Generate bounding boxes for each separate blob in the binary image
// and put them in the 'bboxes' class property
void LaneDetection::getBboxes(const Mat& bin_img, Mat& label_viz_img)
{
  Mat label_img;
  Mat stats;
  Mat centroids; // Not used
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
  // Generate a colorized binary image indicating the different blobs
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
      pixel = colors[label];
    }
  }
#endif
}

// For each blob's bounding box, fit the pixels to a 3rd oder polynomial,
// with the vertical pixel index as the independent variable. The coefficients
// of the curve fits are stored in the 'fit_params' argument
void LaneDetection::fitSegments(Mat& bin_img, std::vector<Eigen::VectorXd>& fit_params, std::vector<Vec4i>& hough_segments)
{
  fit_params.resize(bboxes.size());
  for (size_t i = 0; i < bboxes.size(); i++) {

    // Apply masks to avoid problems when bounding boxes overlap
    Mat masked_img;
    Mat mask = Mat::zeros(bin_img.size(), CV_8U);

    mask(bboxes[i]) = Mat::ones(bboxes[i].height, bboxes[i].width, CV_8U);
    for (size_t j=0; j<bboxes.size(); j++) {
      if (i != j) {
        int j_area = bboxes[j].width * bboxes[j].height;
        int i_area = bboxes[i].width * bboxes[i].height;
        // Mask out the other bounding box if it is smaller than the current one
        if (j_area < i_area){
          mask(bboxes[j]) = Mat::zeros(bboxes[j].height, bboxes[j].width, CV_8U);
        }
      }
    }
    bitwise_and(bin_img, mask, masked_img);

    // Run Canny edge detection on masked image to greatly cut down the number of points
    Mat canny_edge;
    Canny(masked_img(bboxes[i]), canny_edge, 2, 4);

    // Loop through current bounding box and extract pixels whose value
    // is 255 and put their coordinates in 'x_samples' and 'y_samples'
    std::vector<int> x_samples;
    std::vector<int> y_samples;
    for (int xx = 0; xx < canny_edge.cols; xx++) {
      for (int yy = 0; yy < canny_edge.rows; yy+=8) {
        if (canny_edge.at<uint8_t>(Point(xx, yy)) == 255) {
          x_samples.push_back(xx);
          y_samples.push_back(yy);
        }
      }
    }

    // Construct the independent variable sample vector and
    // batch least squares regression matrix
    Eigen::VectorXd x_samples_eig(x_samples.size());
    Eigen::MatrixXd regression_mat(y_samples.size(), 4);
    for (size_t j = 0; j < x_samples.size(); j++) {
      x_samples_eig(j) = (double)x_samples[j];
      regression_mat.row(j) << (double)(y_samples[j] * y_samples[j] * y_samples[j]), (double)(y_samples[j] * y_samples[j]), (double)y_samples[j], 1.0;
    }

    // Compute pseudo-inverse of regression matrix and multiply by the sample vector
    // to compute the least squares curve fit parameters
    Eigen::MatrixXd temp = regression_mat.transpose() * regression_mat;
    Eigen::VectorXd params = temp.inverse() * regression_mat.transpose() * x_samples_eig;

    // Compute normalized absolute error metric to guage the quality of the fit
    double error = 0;
    for (size_t j = 0; j < x_samples.size(); j++) {
      double x_est = params(0) * y_samples[j] * y_samples[j] * y_samples[j] + params(1) * y_samples[j] * y_samples[j] + params(2) * y_samples[j] + params(3);
      error += fabs(x_est - x_samples[j]);
    }
    error /= (double)x_samples.size();

    // If the error is within tolerance, copy parameters into output;
    // otherwise, put an empty vector
    if (error < cfg_.fit_tolerance) {
      fit_params[i] = params;
      continue;
    } else {
      fit_params[i] = Eigen::VectorXd();
    }

    // If it gets here, that means fit error was too big... Use HoughLines
    std::vector<Vec4i> hough_lines;
    cv::HoughLinesP(canny_edge, hough_lines, cfg_.hough_rho_res, cfg_.hough_theta_res, cfg_.hough_threshold, 100, 50);
    if (hough_lines. size() > 0) {
      int max_len_idx = 0;
      int max_len2 = 0;
      for (size_t j=0; j<hough_lines.size(); j++) {
        int len2 = (hough_lines[j][2] - hough_lines[j][0]) * (hough_lines[j][2] - hough_lines[j][0]) + (hough_lines[j][3] - hough_lines[j][1]) * (hough_lines[j][3] - hough_lines[j][1]);
        if (len2 > max_len2) {
          max_len2 = len2;
          max_len_idx = j;
        }
      }

      hough_segments.push_back(Vec4i(hough_lines[max_len_idx][0] + bboxes[i].x, hough_lines[max_len_idx][1] + bboxes[i].y,
                                     hough_lines[max_len_idx][2] + bboxes[i].x, hough_lines[max_len_idx][3] + bboxes[i].y));
    }
  }
}

// For each bounding box, find nearly-horizontal lines with Hough
// and average their rhos and thetas to output a single line
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

    if (hough_lines.size() > 0) {
      avg_line[0] /= (float)hough_lines.size();
      avg_line[1] /= (float)hough_lines.size();
    }
    output.push_back(avg_line);
  }

  return output;
}

// Sample a third order polynomial
int LaneDetection::sampleCurve(const Eigen::VectorXd& params, int y)
{
  return params(0) * y * y * y + params(1) * y * y + params(2) * y + params(3);
}

// This function is called whenever a new image is received from either
// the live running camera driver, a bag file recording, or the simulated
// image coming from Gazebo
void LaneDetection::recvImage(const sensor_msgs::ImageConstPtr& msg)
{
  if (downsample_count < 1) {
    downsample_count++;
    return;
  } else {
    downsample_count = 0;
  }

  // Convert ROS image message into an OpenCV Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  Mat raw_img = cv_ptr->image;

  // Gaussian blur to dampen noise
  cv::GaussianBlur(raw_img, raw_img, Size(cfg_.blur_kernel, cfg_.blur_kernel), 0, 0);

  // Convert to HSV colorspace
  Mat raw_hsv;
  cvtColor(raw_img, raw_hsv, CV_BGR2HSV);

  // Split HSV image into separate single-channel images for H, S, and V
  // and store value and saturation in dedicated variables
  std::vector<Mat> split_img;
  split(raw_hsv, split_img);
  Mat val_img = split_img[2];
  Mat sat_img = split_img[1];

  // Apply threshold to generate a binary value image. White lines have
  // higher value than the road
  Mat val_thres_img;
  threshold(val_img, val_thres_img, cfg_.val_thres, 255, CV_THRESH_BINARY);

  // Apply inverse threshold to generate a binary saturation image. We want
  // to throw out high saturation pixels to avoid detecting grass and orange
  // barrels. White lines and gray road pixels have very low saturation
  Mat sat_thres_img;
  threshold(sat_img, sat_thres_img, cfg_.sat_thres, 255, CV_THRESH_BINARY_INV);

  // AND the two threshold images together to create the binary image to detect lines
  // with
  Mat bin_img;
  bitwise_and(val_thres_img, sat_thres_img, bin_img);

  // Erode
  erode(bin_img, bin_img, Mat::ones(cfg_.erode_size, cfg_.erode_size, CV_8U));

  // Find bounding boxes around each discrete blob in the image
  Mat labeled_image;
  getBboxes(bin_img, labeled_image);

  // Look for horizontal lines to find the stop line
  std::vector<Vec2f> stop_line = detectStopLine(bin_img);

  // Generate curve fits for each discrete blob
  std::vector<Eigen::VectorXd> fit_params;
  std::vector<Vec4i> hough_segments;
  fitSegments(bin_img, fit_params, hough_segments);

  // Sample curve fits and put the sampled points in either
  // the solid line bin or the dashed line bin, based on the
  // size of the bounding box
  std::vector<std::vector<cv::Point> > solid_sampled_points;
  std::vector<std::vector<cv::Point> > dashed_sampled_points;
  for (size_t i = 0; i < fit_params.size(); i++) {

    // If fit error metric was too high, the parameter vector
    // is empty. Skip it.
    if (fit_params[i].rows() == 0) {
      continue;
    }

    // Sample curve inside bounding box at y pixel values with
    // resolution specified in the 'reconstruct_pix' parameter
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

    // If the largest dimension of the bounding box is greater than
    // 'dashed_length' threshold, add it to the solid bin, otherwise
    // add it to the dashed bin
    int largest_dim = std::max(bboxes[i].height, bboxes[i].width);
    if (largest_dim > cfg_.dashed_length) {
      solid_sampled_points.push_back(segment_points);
    } else {
      dashed_sampled_points.push_back(segment_points);
    }
  }

  for (size_t i=0; i < hough_segments.size(); i++) {
    std::vector<cv::Point> segment_points;
    cv::Point new_point;
    for (int y = hough_segments[i][1]; y <= hough_segments[i][3]; y += cfg_.reconstruct_pix) {
      float slope = (float)(hough_segments[i][2] - hough_segments[i][0]) / (float)(hough_segments[i][3] - hough_segments[i][1]);
      new_point.x = slope * (y - hough_segments[i][1]) + hough_segments[i][0];
      new_point.y = y;
      segment_points.push_back(new_point);
    }
    new_point.x = hough_segments[i][2];
    new_point.y = hough_segments[i][3];
    segment_points.push_back(new_point);
    solid_sampled_points.push_back(segment_points);
  }

#if DEBUG
  // Show colorized binary image
  imshow("Binary", labeled_image);
  waitKey(1);

  // Draw green boxes to show the bounding boxes of the discrete blobs
  for (size_t i = 0; i < bboxes.size(); i++) {
    cv::rectangle(raw_img, bboxes[i], cv::Scalar(0, 255, 0));
  }

  // Draw red lines to show solid line curve fits
  for (size_t i = 0; i < solid_sampled_points.size(); i++) {
    for (size_t j = 1; j < solid_sampled_points[i].size(); j++) {
      cv::line(raw_img, solid_sampled_points[i][j - 1], solid_sampled_points[i][j], cv::Scalar(0, 0, 255));
    }
  }

  // Draw cyan lines to show dashed line curve fits
  for (size_t i = 0; i < dashed_sampled_points.size(); i++) {
    for (size_t j = 1; j < dashed_sampled_points[i].size(); j++) {
      cv::line(raw_img, dashed_sampled_points[i][j - 1], dashed_sampled_points[i][j], cv::Scalar(255, 255, 0));
    }
  }

  // Convert rhos and thetas from detected stop lines into a line
  // and show it as a blue line in the image
  for (size_t i=0; i<stop_line.size(); i++) {
    float rho = stop_line[i][0];
    if (rho > 0) {
      float t_theta = tan(stop_line[i][1]);
      float s_theta = sin(stop_line[i][1]);
      Point p1, p2;
      p1.x = bboxes[i].x;
      p1.y = rho / s_theta - p1.x / t_theta + bboxes[i].y;
      p2.x = bboxes[i].x + bboxes[i].width;
      p2.y = rho / s_theta - p2.x / t_theta + bboxes[i].y;
      line(raw_img, p1, p2, cv::Scalar(255, 0, 0));
    }
  }

#endif

  // Project output lines from camera into vehicle frame and publish as obstacles
  tf::StampedTransform transform;
  try {
    listener_.lookupTransform("base_footprint", msg->header.frame_id, msg->header.stamp, transform);
  } catch (tf::TransformException& ex) {
    ROS_WARN_THROTTLE(1.0, "%s", ex.what());
    return;
  }

  // Create pinhole camera model instance and load
  // its parameters from the camera info
  // generated using the checkerboard calibration program
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info_);

  // Compute the camera's line of sight vector in vehicle frame
  tf::Vector3 cam_los_vect = transform(tf::Vector3(0, 0, 0));

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

  // Project solid line points
  sensor_msgs::PointCloud solid_line_points;
  solid_line_points.header.frame_id = "base_footprint";
  solid_line_points.header.stamp = msg->header.stamp;
  for (size_t i = 0; i < solid_sampled_points.size(); i++) {
    for (size_t j = 1; j < solid_sampled_points[i].size(); j++) {
      // Project pixel points into 3D points in vehicle frame
      geometry_msgs::Point32 p1 = projectPoint(model, transform, cam_los_vect, solid_sampled_points[i][j - 1]);
      geometry_msgs::Point32 p2 = projectPoint(model, transform, cam_los_vect, solid_sampled_points[i][j]);

      // Add point to solid line point cloud for output
      solid_line_points.points.push_back(p1);

      // Add points to visualize them in Rviz
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
    geometry_msgs::Point32 last_point = projectPoint(model, transform, cam_los_vect, solid_sampled_points[i].back());
    solid_line_points.points.push_back(last_point);
  }

  sensor_msgs::PointCloud dashed_line_points;
  dashed_line_points.header.frame_id = "base_footprint";
  dashed_line_points.header.stamp = msg->header.stamp;
  for (size_t i = 0; i < dashed_sampled_points.size(); i++) {
    for (size_t j = 1; j < dashed_sampled_points[i].size(); j++) {
      // Project pixel points into 3D points in vehicle frame
      geometry_msgs::Point32 p1 = projectPoint(model, transform, cam_los_vect, dashed_sampled_points[i][j - 1]);
      geometry_msgs::Point32 p2 = projectPoint(model, transform, cam_los_vect, dashed_sampled_points[i][j]);
      dashed_line_points.points.push_back(p1);

      // Add points to visualize them in Rviz
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
    geometry_msgs::Point32 last_point = projectPoint(model, transform, cam_los_vect, dashed_sampled_points[i].back());
    dashed_line_points.points.push_back(last_point);
  }

  // Find distance to closest stop line
  int max_y = 0;
  int x = bin_img.cols / 2;
  for (size_t i=0; i<stop_line.size(); i++) {
    float rho = stop_line[i][0];
    float theta = stop_line[i][1];
    if (rho > 0) {
      int y = (int)(rho / sin(theta) - x / tan(theta)) + bboxes[i].y;
      if (y > max_y) {
        max_y = y;
      }
    }
  }
#if DEBUG
  if (max_y > 0) {
    cv::rectangle(raw_img, Rect(x-5, max_y-5, 10, 10), Scalar(255,255,255));
  }

  // Show output image with all the lines and boxes
  imshow("Output Image", raw_img);
  waitKey(1);
#endif

  std_msgs::Float64 stop_line_dist_msg;
  if (max_y == 0) {
    stop_line_dist_msg.data = INFINITY;
  } else {
    geometry_msgs::Point32 stop_line_point = projectPoint(model, transform, cam_los_vect, Point(x, max_y));
    stop_line_dist_msg.data = stop_line_point.x;
  }

  pub_solid_line_cloud_.publish(solid_line_points);
  pub_dashed_line_cloud_.publish(dashed_line_points);
  pub_line_visualization_.publish(viz_marker);
  pub_stop_line_dist_.publish(stop_line_dist_msg);
}

// Project 2D pixel point 'p' into vehicle's frame and return as 3D point
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

}
