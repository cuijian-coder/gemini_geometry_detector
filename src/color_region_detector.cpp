#include "gemini_geometry_detector/color_region_detector.h"

#include "gemini_geometry_detector/fit_line_guide_line_estimator.h"
#include "gemini_geometry_detector/depth_guide_line_estimator.h"
#include "gemini_geometry_detector/topic_ground_plane_provider.h"
#include "gemini_geometry_detector/imu_ground_plane_provider.h"

#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace gemini_geometry_detector
{

ColorRegionDetector::ColorRegionDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), it_(nh),
    tf_listener_(tf_buffer_),
    frame_counter_(0),
    first_rgb_received_(false),
    use_tf_target_angle_(true),
    target_angle_computed_(false),
    camera_info_received_(false)
{
  loadParameters();

  if (guide_line_estimator_type_ == "FitLineGuideLineEstimator")
  {
    guide_line_estimator_.reset(new FitLineGuideLineEstimator());
    ROS_INFO("Using FitLineGuideLineEstimator (RGB-only image-plane fitting).");
  }
  else
  {
    guide_line_estimator_.reset(new DepthGuideLineEstimator());
    ROS_INFO("Using DepthGuideLineEstimator (ground-plane 3D fitting).");
  }

  if (guide_line_estimator_type_ == "DepthGuideLineEstimator")
  {
    if (ground_plane_provider_type_ == "imu")
    {
      ground_plane_provider_.reset(new ImuGroundPlaneProvider());
      ROS_INFO("Using ImuGroundPlaneProvider (IMU gravity + fixed camera height).");
    }
    else
    {
      ground_plane_provider_.reset(new TopicGroundPlaneProvider());
      ROS_INFO("Using TopicGroundPlaneProvider (subscribed ground-plane topic).");
    }
    ground_plane_provider_->configure(pnh_);
  }

  mask_pub_ = it_.advertise("color/mask", 1);
  annotated_pub_ = it_.advertise("color/annotated", 1);
  contours_pub_ = nh_.advertise<ContourArray>("color/contours", 1);
  guide_line_error_pub_ = nh_.advertise<GuideLineError>("color/guide_line_error", 1);

  rgb_sub_ = it_.subscribe(input_topic_, 1,
                           &ColorRegionDetector::rgbCallback, this);
  camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1,
                                   &ColorRegionDetector::cameraInfoCallback, this);

  if (ground_plane_provider_)
  {
    ground_plane_provider_->start(nh_,
                                  [this](const Eigen::Vector3f& normal, float d)
                                  {
                                    guide_line_estimator_->setGroundPlane(normal, d);
                                    ROS_INFO_THROTTLE(5.0,
                                                      "Ground plane set: "
                                                      "n=(%.3f, %.3f, %.3f), d=%.3f",
                                                      normal.x(), normal.y(),
                                                      normal.z(), d);
                                  });
  }

  ROS_INFO("============================================================");
  ROS_INFO("ColorRegionDetector started");
  ROS_INFO("------------------------------------------------------------");
  ROS_INFO("[Input/output]");
  ROS_INFO("  RGB topic:        %s", input_topic_.c_str());
  ROS_INFO("  CameraInfo topic: %s", camera_info_topic_.c_str());
  ROS_INFO("  Output namespace: gemini_geometry_detector/color/*");
  ROS_INFO("[TF configuration]");
  ROS_INFO("  base_frame:          %s", base_frame_.c_str());
  ROS_INFO("  camera_frame:        %s", camera_frame_.c_str());
  ROS_INFO("  use_tf_target_angle: %s", use_tf_target_angle_ ? "true" : "false");
  ROS_INFO("[Guide-line estimator]");
  ROS_INFO("  type:         %s", guide_line_estimator_type_.c_str());
  ROS_INFO("  target_angle: %.4f rad (%.1f deg)",
           target_angle_, target_angle_ * 180.0 / M_PI);
  if (guide_line_estimator_type_ == "DepthGuideLineEstimator")
  {
    ROS_INFO("  look_ahead_m: %.3f m", look_ahead_m_);
    ROS_INFO("[Ground-plane provider]");
    ROS_INFO("  type: %s", ground_plane_provider_type_.c_str());
  }
  ROS_INFO("[Image preprocessing]");
  ROS_INFO("  image_scale:        %.2f", image_scale_);
  ROS_INFO("  roi_y_ratio:        %.2f", roi_y_ratio_);
  ROS_INFO("  roi_y:              %d (%s)", roi_y_,
           roi_y_ < 0 ? "use roi_y_ratio" : "absolute pixels");
  ROS_INFO("[Color segmentation (HSV)]");
  ROS_INFO("  H: [%3d, %3d]", h_min_, h_max_);
  ROS_INFO("  S: [%3d, %3d]", s_min_, s_max_);
  ROS_INFO("  V: [%3d, %3d]", v_min_, v_max_);
  ROS_INFO("[Contour filtering]");
  ROS_INFO("  morph_kernel_size:  %d", morph_kernel_size_);
  ROS_INFO("  min_contour_area:   %d", min_contour_area_);
  ROS_INFO("  max_contour_points: %d", max_contour_points_);
  ROS_INFO("============================================================");
}

void ColorRegionDetector::loadParameters()
{
  pnh_.param<std::string>("input_topic", input_topic_, "/camera/color/image_raw");
  pnh_.param<std::string>("camera_info_topic", camera_info_topic_, "/camera/color/camera_info");
  pnh_.param<std::string>("base_frame", base_frame_, "base_link");
  pnh_.param<std::string>("camera_frame", camera_frame_, "camera_color_optical_frame");

  pnh_.param("h_min", h_min_, 20);
  pnh_.param("h_max", h_max_, 40);
  pnh_.param("s_min", s_min_, 80);
  pnh_.param("s_max", s_max_, 255);
  pnh_.param("v_min", v_min_, 80);
  pnh_.param("v_max", v_max_, 255);

  pnh_.param("morph_kernel_size", morph_kernel_size_, 5);
  pnh_.param("min_contour_area", min_contour_area_, 200);
  pnh_.param("max_contour_points", max_contour_points_, 64);

  pnh_.param("image_scale", image_scale_, 0.5);
  pnh_.param("use_tf_target_angle", use_tf_target_angle_, true);
  pnh_.param("target_angle", target_angle_, -M_PI / 2.0);
  pnh_.param("roi_y_ratio", roi_y_ratio_, 0.75);
  pnh_.param("roi_y", roi_y_, -1);
  pnh_.param("look_ahead_m", look_ahead_m_, 0.0);
  pnh_.param<std::string>("guide_line_estimator_type",
                          guide_line_estimator_type_,
                          "DepthGuideLineEstimator");
  pnh_.param<std::string>("ground_plane_provider_type",
                          ground_plane_provider_type_,
                          "topic");

  if (image_scale_ <= 0.0 || image_scale_ > 1.0)
  {
    ROS_WARN("image_scale must be in (0, 1], reset to 1.0");
    image_scale_ = 1.0;
  }
  if (roi_y_ratio_ < 0.0 || roi_y_ratio_ > 1.0)
  {
    ROS_WARN("roi_y_ratio must be in [0, 1], reset to 0.75");
    roi_y_ratio_ = 0.75;
  }

  if (morph_kernel_size_ > 0 && morph_kernel_size_ % 2 == 0)
  {
    morph_kernel_size_ += 1;
    ROS_WARN("morph_kernel_size must be odd, adjusted to %d", morph_kernel_size_);
  }

  if (look_ahead_m_ < 0.0)
  {
    ROS_WARN("look_ahead_m must be >= 0, reset to 0.0");
    look_ahead_m_ = 0.0;
  }

  if (guide_line_estimator_type_ != "FitLineGuideLineEstimator" &&
      guide_line_estimator_type_ != "DepthGuideLineEstimator")
  {
    ROS_WARN("Unknown guide_line_estimator_type '%s', using DepthGuideLineEstimator",
             guide_line_estimator_type_.c_str());
    guide_line_estimator_type_ = "DepthGuideLineEstimator";
  }

  if (ground_plane_provider_type_ != "topic" &&
      ground_plane_provider_type_ != "imu")
  {
    ROS_WARN("Unknown ground_plane_provider_type '%s', using topic",
             ground_plane_provider_type_.c_str());
    ground_plane_provider_type_ = "topic";
  }
}

void ColorRegionDetector::rgbCallback(const sensor_msgs::ImageConstPtr& rgb_msg)
{
  if (!first_rgb_received_)
  {
    ROS_INFO("First RGB frame received: %dx%d", rgb_msg->width, rgb_msg->height);
    first_rgb_received_ = true;
  }

  cv_bridge::CvImageConstPtr cv_rgb;
  try
  {
    cv_rgb = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  processFrame(rgb_msg, cv_rgb->image);
}

void ColorRegionDetector::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  if (camera_info_received_)
  {
    return;
  }

  const auto& k = info_msg->K;

  camera_intrinsics_.fx = static_cast<float>(k[0]);
  camera_intrinsics_.fy = static_cast<float>(k[4]);
  camera_intrinsics_.cx = static_cast<float>(k[2]);
  camera_intrinsics_.cy = static_cast<float>(k[5]);

  camera_info_received_ = true;

  guide_line_estimator_->configure(camera_intrinsics_, image_scale_, target_angle_,
                                   roi_y_ratio_, roi_y_, look_ahead_m_);

  ROS_INFO("CameraInfo received");
  ROS_INFO("  raw intrinsics: fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
           camera_intrinsics_.fx, camera_intrinsics_.fy,
           camera_intrinsics_.cx, camera_intrinsics_.cy);
  ROS_INFO("  image_scale:    %.2f", image_scale_);
  if (guide_line_estimator_type_ == "DepthGuideLineEstimator")
  {
    ROS_INFO("  Depth estimator will use scaled intrinsics: "
             "fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
             camera_intrinsics_.fx * image_scale_,
             camera_intrinsics_.fy * image_scale_,
             camera_intrinsics_.cx * image_scale_,
             camera_intrinsics_.cy * image_scale_);
  }
}

void ColorRegionDetector::processFrame(const sensor_msgs::ImageConstPtr& rgb_msg,
                                       const cv::Mat& bgr_image)
{
  ++frame_counter_;

  ensureTargetAngleComputed();

  cv::Mat scaled_bgr;
  const cv::Mat* proc_image = &bgr_image;
  if (image_scale_ < 1.0)
  {
    cv::resize(bgr_image, scaled_bgr, cv::Size(), image_scale_, image_scale_, cv::INTER_LINEAR);
    proc_image = &scaled_bgr;
  }

  cv::Mat mask = createColorMask(*proc_image);
  applyMorphology(mask);

  std::vector<std::vector<cv::Point>> contours;
  detectContours(mask, contours);

  cv::Mat annotated = proc_image->clone();
  ContourArray contour_array;
  contour_array.header = rgb_msg->header;

  const double scaled_min_area = static_cast<double>(min_contour_area_) *
                                 image_scale_ * image_scale_;

  int id = 0;
  int filtered_count = 0;

  for (const auto& contour : contours)
  {
    ContourInfo info;
    if (!buildContourInfo(contour, id, info, scaled_min_area))
    {
      ++filtered_count;
      continue;
    }

    drawContourOnImage(annotated, contour, info);
    contour_array.contours.push_back(info);
    ++id;
  }

  const auto largest_contour = findLargestContour(contours, scaled_min_area);
  const auto rect_contour = getRectangularContour(largest_contour);
  const double largest_area = largest_contour.empty()
                                  ? 0.0
                                  : cv::contourArea(largest_contour);

  GuideLineError error_msg;
  std::string invalid_reason;
  if (!camera_intrinsics_.valid())
  {
    invalid_reason = "no valid camera_info";
  }
  else if (largest_contour.empty())
  {
    invalid_reason = "no valid contour";
  }

  if (!invalid_reason.empty())
  {
    error_msg.header = rgb_msg->header;
    error_msg.valid = false;
    error_msg.yaw_error = 0.0;
    error_msg.lateral_error_n = 0.0;
    error_msg.lateral_error_m = 0.0;
    error_msg.roi_point.x = 0.0;
    error_msg.roi_point.y = 0.0;
    error_msg.roi_point.z = 0.0;

    ROS_WARN_THROTTLE(1.0,
                      "GuideLineError invalid: %s (raw contours: %zu, "
                      "scaled_min_area: %.1f, image: %dx%d)",
                      invalid_reason.c_str(), contours.size(), scaled_min_area,
                      proc_image->cols, proc_image->rows);
  }
  else
  {
    error_msg = guide_line_estimator_->estimate(largest_contour, rect_contour,
                                                proc_image->size(), rgb_msg->header,
                                                annotated);
    if (!error_msg.valid)
    {
      ROS_WARN_THROTTLE(1.0,
                        "GuideLineError invalid: estimator returned invalid result "
                        "(see estimator logs for details)");
    }
  }

  guide_line_error_pub_.publish(error_msg);

  if (error_msg.valid)
  {
    ROS_INFO_THROTTLE(1.0,
                      "[Frame %d] GuideLineError OK: "
                      "yaw=%.4f rad (%.1f deg), lat_n=%.4f, lat_m=%.4f, "
                      "roi=(%.1f, %.1f)",
                      frame_counter_,
                      error_msg.yaw_error,
                      error_msg.yaw_error * 180.0 / M_PI,
                      error_msg.lateral_error_n,
                      error_msg.lateral_error_m,
                      error_msg.roi_point.x,
                      error_msg.roi_point.y);
  }
  else
  {
    ROS_WARN_THROTTLE(1.0,
                      "[Frame %d] GuideLineError INVALID: reason=%s, "
                      "camera_info=%s, largest_contour=%s, "
                      "raw contours=%zu, scaled_min_area=%.1f",
                      frame_counter_,
                      invalid_reason.empty() ? "estimator" : invalid_reason.c_str(),
                      camera_info_received_ ? "yes" : "no",
                      largest_contour.empty() ? "empty" : "present",
                      contours.size(),
                      scaled_min_area);
  }

  ROS_INFO_THROTTLE(1.0,
                    "[Frame %d] Detection: image=%dx%d scale=%.2f, "
                    "raw_contours=%zu, valid_2d=%zu, filtered=%d, largest_area=%.1f",
                    frame_counter_,
                    proc_image->cols, proc_image->rows, image_scale_,
                    contours.size(), contour_array.contours.size(), filtered_count,
                    largest_area);

  publishResults(rgb_msg->header, mask, annotated, contour_array);
}

std::vector<cv::Point> ColorRegionDetector::findLargestContour(
    const std::vector<std::vector<cv::Point>>& contours,
    double min_area) const
{
  std::vector<cv::Point> largest;
  double max_area = 0.0;
  for (const auto& contour : contours)
  {
    if (contour.size() < 3)
    {
      continue;
    }
    const double area = cv::contourArea(contour);
    if (area < min_area || area <= 1e-6)
    {
      continue;
    }
    if (area > max_area)
    {
      max_area = area;
      largest = contour;
    }
  }
  return largest;
}

std::vector<cv::Point> ColorRegionDetector::getRectangularContour(
    const std::vector<cv::Point>& contour) const
{
  std::vector<cv::Point> rect;
  if (contour.size() < 3)
  {
    return rect;
  }

  const double area = cv::contourArea(contour);
  if (area <= 1e-6)
  {
    return rect;
  }

  try
  {
    std::vector<cv::Point2f> pts(contour.begin(), contour.end());
    cv::RotatedRect rotated_rect = cv::minAreaRect(pts);
    if (rotated_rect.size.width <= 0.0f || rotated_rect.size.height <= 0.0f)
    {
      return rect;
    }

    cv::Mat box_mat;
    cv::boxPoints(rotated_rect, box_mat);

    std::vector<cv::Point2f> box_float;
    box_float.reserve(4);
    for (int i = 0; i < box_mat.rows; ++i)
    {
      box_float.emplace_back(box_mat.at<float>(i, 0), box_mat.at<float>(i, 1));
    }

    cv::Point2f center = rotated_rect.center;
    std::sort(box_float.begin(), box_float.end(),
              [&center](const cv::Point2f& a, const cv::Point2f& b)
              {
                const float angle_a = std::atan2(a.y - center.y, a.x - center.x);
                const float angle_b = std::atan2(b.y - center.y, b.x - center.x);
                return angle_a < angle_b;
              });

    rect.reserve(box_float.size());
    for (const auto& p : box_float)
    {
      rect.emplace_back(static_cast<int>(std::round(p.x)), static_cast<int>(std::round(p.y)));
    }
  }
  catch (const cv::Exception& e)
  {
    ROS_WARN_THROTTLE(1.0, "getRectangularContour failed: %s", e.what());
    rect.clear();
  }

  return rect;
}

bool ColorRegionDetector::computeTargetAngleFromTf()
{
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform(camera_frame_, base_frame_,
                                           ros::Time(0), ros::Duration(0.5));
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(5.0,
                      "TF lookup from %s to %s failed: %s",
                      base_frame_.c_str(), camera_frame_.c_str(), ex.what());
    return false;
  }

  // Vehicle forward direction in base_link is +X.
  const Eigen::Vector3d forward_base(1.0, 0.0, 0.0);

  // Rotate into camera optical frame. Direction vectors only need rotation.
  const Eigen::Quaterniond q(transform.transform.rotation.w,
                             transform.transform.rotation.x,
                             transform.transform.rotation.y,
                             transform.transform.rotation.z);
  const Eigen::Vector3d forward_cam = q * forward_base;

  if (std::abs(forward_cam.z()) < 1e-6)
  {
    ROS_WARN_THROTTLE(5.0,
                      "Vehicle forward direction is parallel to the image plane");
    return false;
  }

  // Normalized image-plane projection. The angle is independent of fx/fy.
  const double x_n = forward_cam.x() / forward_cam.z();
  const double y_n = forward_cam.y() / forward_cam.z();

  target_angle_ = std::atan2(y_n, x_n);
  target_angle_computed_ = true;

  // Re-configure the estimator with the newly computed target angle.
  if (guide_line_estimator_ && camera_intrinsics_.valid())
  {
    guide_line_estimator_->configure(camera_intrinsics_, image_scale_, target_angle_,
                                     roi_y_ratio_, roi_y_, look_ahead_m_);
  }

  ROS_INFO("Computed target_angle from TF:");
  ROS_INFO("  target_angle: %.4f rad (%.1f deg)",
           target_angle_, target_angle_ * 180.0 / M_PI);
  ROS_INFO("  forward_cam in optical frame: (%.3f, %.3f, %.3f)",
           forward_cam.x(), forward_cam.y(), forward_cam.z());
  return true;
}

void ColorRegionDetector::ensureTargetAngleComputed()
{
  if (!use_tf_target_angle_ || target_angle_computed_)
  {
    return;
  }

  if (!computeTargetAngleFromTf())
  {
    ROS_WARN_THROTTLE(5.0,
                      "Failed to compute target_angle from TF. Using fallback "
                      "target_angle=%.4f rad from config.",
                      target_angle_);
  }
}

cv::Mat ColorRegionDetector::createColorMask(const cv::Mat& bgr_image)
{
  cv::Mat hsv;
  cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), mask);
  return mask;
}

void ColorRegionDetector::applyMorphology(cv::Mat& mask)
{
  if (morph_kernel_size_ <= 0)
  {
    return;
  }

  cv::Mat element = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(morph_kernel_size_, morph_kernel_size_));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);
}

void ColorRegionDetector::detectContours(const cv::Mat& mask,
                                         std::vector<std::vector<cv::Point>>& contours)
{
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
}

bool ColorRegionDetector::buildContourInfo(const std::vector<cv::Point>& contour,
                                           int id,
                                           ContourInfo& info,
                                           double min_contour_area)
{
  double area = cv::contourArea(contour);
  if (area < min_contour_area)
  {
    return false;
  }

  cv::Moments m = cv::moments(contour);
  cv::Point2f center(0, 0);
  if (m.m00 > 0)
  {
    center.x = static_cast<float>(m.m10 / m.m00);
    center.y = static_cast<float>(m.m01 / m.m00);
  }

  cv::Rect bbox = cv::boundingRect(contour);

  info.id = id;
  info.center.x = center.x;
  info.center.y = center.y;
  info.center.z = 0.0;
  info.area = area;
  info.bbox_tl.x = bbox.x;
  info.bbox_tl.y = bbox.y;
  info.bbox_tl.z = 0.0;
  info.bbox_br.x = bbox.x + bbox.width;
  info.bbox_br.y = bbox.y + bbox.height;
  info.bbox_br.z = 0.0;

  size_t step = 1;
  if (max_contour_points_ > 0 && contour.size() > static_cast<size_t>(max_contour_points_))
  {
    step = contour.size() / max_contour_points_;
    if (step < 1)
    {
      step = 1;
    }
  }

  info.points.clear();
  for (size_t i = 0; i < contour.size(); i += step)
  {
    geometry_msgs::Point32 p;
    p.x = contour[i].x;
    p.y = contour[i].y;
    p.z = 0.0f;
    info.points.push_back(p);
  }

  return true;
}

void ColorRegionDetector::drawContourOnImage(cv::Mat& annotated,
                                             const std::vector<cv::Point>& contour,
                                             const ContourInfo& info)
{
  cv::Rect bbox(
      static_cast<int>(info.bbox_tl.x),
      static_cast<int>(info.bbox_tl.y),
      static_cast<int>(info.bbox_br.x - info.bbox_tl.x),
      static_cast<int>(info.bbox_br.y - info.bbox_tl.y));

  cv::drawContours(annotated, std::vector<std::vector<cv::Point>>{contour}, -1,
                   cv::Scalar(0, 255, 0), 2);
  cv::circle(annotated,
             cv::Point(static_cast<int>(info.center.x), static_cast<int>(info.center.y)),
             4, cv::Scalar(0, 0, 255), -1);
  cv::rectangle(annotated, bbox, cv::Scalar(255, 0, 0), 2);
}

void ColorRegionDetector::publishResults(const std_msgs::Header& header,
                                         const cv::Mat& mask,
                                         const cv::Mat& annotated,
                                         const ContourArray& contour_array)
{
  ROS_INFO_THROTTLE(1.0,
                    "Publishing: mask + annotated + %zu 2D contours",
                    contour_array.contours.size());

  cv_bridge::CvImage mask_msg;
  mask_msg.header = header;
  mask_msg.encoding = sensor_msgs::image_encodings::MONO8;
  mask_msg.image = mask;
  mask_pub_.publish(mask_msg.toImageMsg());

  cv_bridge::CvImage annotated_msg;
  annotated_msg.header = header;
  annotated_msg.encoding = sensor_msgs::image_encodings::BGR8;
  annotated_msg.image = annotated;
  annotated_pub_.publish(annotated_msg.toImageMsg());

  contours_pub_.publish(contour_array);
}

}  // namespace gemini_geometry_detector
