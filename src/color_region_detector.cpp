#include "gemini_geometry_detector/color_region_detector.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <Eigen/Dense>
#include <cmath>

namespace gemini_geometry_detector
{

ColorRegionDetector::ColorRegionDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), it_(nh),
    guide_line_estimator_(pnh),
    frame_counter_(0),
    first_sync_received_(false),
    ground_normal_(Eigen::Vector3f::UnitZ()),
    ground_d_(0.0f),
    ground_plane_received_(false)
{
  loadParameters();

  mask_pub_ = it_.advertise("color/mask", 1);
  annotated_pub_ = it_.advertise("color/annotated", 1);
  contours_pub_ = nh_.advertise<ContourArray>("color/contours", 1);
  depth_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("color/depth_cloud", 1);
  guide_line_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("color/guide_line_cloud", 1);
  guide_line_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("color/guide_line_marker", 1);
  contours_3d_pub_ = nh_.advertise<Contour3DArray>("color/contours_3d", 1);

  rgb_sub_filter_.subscribe(it_, input_topic_, 1);
  cloud_sub_.subscribe(nh_, point_cloud_topic_, 1);

  cloud_sync_.reset(new message_filters::Synchronizer<CloudSyncPolicy>(
      CloudSyncPolicy(10), rgb_sub_filter_, cloud_sub_));
  cloud_sync_->registerCallback(boost::bind(&ColorRegionDetector::rgbCloudCallback, this, _1, _2));

  camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1,
                                   &ColorRegionDetector::cameraInfoCallback, this);
  plane_coefficients_sub_ = nh_.subscribe(plane_coefficients_topic_, 1,
                                          &ColorRegionDetector::planeCoefficientsCallback, this);

  ROS_INFO("ColorRegionDetector started.");
  ROS_INFO("RGB topic: %s", input_topic_.c_str());
  ROS_INFO("Point cloud topic: %s", point_cloud_topic_.c_str());
  ROS_INFO("CameraInfo topic: %s", camera_info_topic_.c_str());
  ROS_INFO("Plane coefficients topic: %s", plane_coefficients_topic_.c_str());
  ROS_INFO("HSV range: H[%d,%d] S[%d,%d] V[%d,%d]", h_min_, h_max_, s_min_, s_max_, v_min_, v_max_);
}

void ColorRegionDetector::loadParameters()
{
  pnh_.param<std::string>("input_topic", input_topic_, "/camera/color/image_raw");
  pnh_.param<std::string>("point_cloud_topic", point_cloud_topic_, "/camera/depth_registered/points");
  pnh_.param<std::string>("camera_info_topic", camera_info_topic_, "/camera/color/camera_info");
  pnh_.param<std::string>("plane_coefficients_topic", plane_coefficients_topic_,
                          "/ground_plane_calibrator/ground_plane/coefficients");

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
  pnh_.param("guide_line_every_n", guide_line_every_n_, 1);

  if (image_scale_ <= 0.0 || image_scale_ > 1.0)
  {
    ROS_WARN("image_scale must be in (0, 1], reset to 1.0");
    image_scale_ = 1.0;
  }
  if (guide_line_every_n_ < 1)
  {
    guide_line_every_n_ = 1;
  }

  if (morph_kernel_size_ > 0 && morph_kernel_size_ % 2 == 0)
  {
    morph_kernel_size_ += 1;
    ROS_WARN("morph_kernel_size must be odd, adjusted to %d", morph_kernel_size_);
  }
}

void ColorRegionDetector::rgbCloudCallback(const sensor_msgs::ImageConstPtr& rgb_msg,
                                           const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  if (!first_sync_received_)
  {
    ROS_INFO("First synced frame received: RGB %dx%d, Cloud %dx%d, frame_id: %s",
             rgb_msg->width, rgb_msg->height,
             cloud_msg->width, cloud_msg->height,
             cloud_msg->header.frame_id.c_str());
    first_sync_received_ = true;
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

  processFrame(rgb_msg, cv_rgb->image, cloud_msg);
}

void ColorRegionDetector::planeCoefficientsCallback(
    const ground_plane_calibrator::PlaneCoefficientsConstPtr& msg)
{
  ground_normal_ = Eigen::Vector3f(static_cast<float>(msg->A),
                                   static_cast<float>(msg->B),
                                   static_cast<float>(msg->C));
  ground_d_ = static_cast<float>(msg->D);
  ground_plane_received_ = true;

  ROS_INFO("Received ground plane coefficients: A=%.6f B=%.6f C=%.6f D=%.6f",
           msg->A, msg->B, msg->C, msg->D);
}

void ColorRegionDetector::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  if (image_scale_ >= 1.0)
  {
    guide_line_estimator_.setCameraInfo(info_msg);
  }
  else
  {
    sensor_msgs::CameraInfoPtr scaled(new sensor_msgs::CameraInfo(*info_msg));
    scaled->K[0] *= image_scale_;  // fx
    scaled->K[2] *= image_scale_;  // cx
    scaled->K[4] *= image_scale_;  // fy
    scaled->K[5] *= image_scale_;  // cy
    scaled->width = static_cast<uint32_t>(static_cast<double>(scaled->width) * image_scale_);
    scaled->height = static_cast<uint32_t>(static_cast<double>(scaled->height) * image_scale_);
    guide_line_estimator_.setCameraInfo(scaled);
  }

  ROS_INFO_ONCE("CameraInfo received: fx=%.3f fy=%.3f cx=%.3f cy=%.3f (scale=%.2f)",
                info_msg->K[0], info_msg->K[4], info_msg->K[2], info_msg->K[5],
                image_scale_);
}

void ColorRegionDetector::processFrame(const sensor_msgs::ImageConstPtr& rgb_msg,
                                       const cv::Mat& bgr_image,
                                       const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ++frame_counter_;

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

  depth_cloud_pub_.publish(*cloud_msg);

  // Estimate 3D guide line from the rectangular approximation of the largest contour.
  const bool compute_guide_line = (guide_line_every_n_ <= 1) ||
                                  ((frame_counter_ % guide_line_every_n_) == 0);
  const auto largest_contour = findLargestContour(contours, scaled_min_area);
  const auto rect_contour = getRectangularContour(largest_contour);

  if (!rect_contour.empty())
  {
    // Draw the rectangular approximation on the annotated image.
    cv::polylines(annotated, std::vector<std::vector<cv::Point>>{rect_contour},
                  true, cv::Scalar(0, 255, 255), 2);
  }
  else if (!largest_contour.empty())
  {
    ROS_WARN_THROTTLE(1.0,
                      "Largest contour found but failed to convert to rectangle (area=%.1f)",
                      cv::contourArea(largest_contour));
  }
  else if (compute_guide_line)
  {
    ROS_WARN_THROTTLE(1.0,
                      "No valid contour for guide line (raw contours: %zu, "
                      "scaled_min_area: %.1f)",
                      contours.size(), scaled_min_area);
  }

  if (compute_guide_line && !rect_contour.empty())
  {
    if (!guide_line_estimator_.hasCameraInfo())
    {
      ROS_WARN_THROTTLE(1.0, "Cannot estimate guide line: camera_info not received yet");
    }
    else if (!ground_plane_received_)
    {
      ROS_WARN_THROTTLE(1.0,
                        "Cannot estimate guide line: ground plane A/B/C/D not received yet "
                        "(topic: %s)",
                        plane_coefficients_topic_.c_str());
    }
    else
    {
      std::vector<cv::Point> sampled_contour;
      std::vector<Eigen::Vector3f> guide_line;
      if (guide_line_estimator_.estimate(rect_contour, ground_normal_, ground_d_,
                                         sampled_contour, guide_line))
      {
        std::vector<Eigen::Vector3f> rectangle;
        if (guide_line_estimator_.fitRectangleOnPlane(guide_line, ground_normal_, ground_d_,
                                                      rectangle))
        {
          const std::string& frame_id = cloud_msg->header.frame_id;
          guide_line_cloud_pub_.publish(
              guide_line_estimator_.buildPointCloud2(rgb_msg->header, frame_id, rectangle));
          guide_line_marker_pub_.publish(
              guide_line_estimator_.buildLineMarker(rgb_msg->header, frame_id, rectangle));

          const double area = cv::contourArea(rect_contour);
          contours_3d_pub_.publish(
              guide_line_estimator_.buildContour3DArray(rgb_msg->header, rect_contour,
                                                         rectangle, area));

          ROS_INFO_THROTTLE(1.0, "Guide line published");
        }
        else
        {
          ROS_WARN_THROTTLE(1.0, "Failed to fit rectangle on ground plane");
        }
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "Failed to estimate guide line from rectangular contour");
      }
    }
  }
  else if (!compute_guide_line && !rect_contour.empty())
  {
    ROS_INFO_THROTTLE(1.0,
                      "Skipping guide line this frame (frame_counter=%d, every_n=%d)",
                      frame_counter_, guide_line_every_n_);
  }

  ROS_INFO_THROTTLE(1.0,
                    "Image %dx%d (scale=%.2f), raw contours: %zu, valid 2D: %zu, filtered: %d",
                    proc_image->cols, proc_image->rows, image_scale_,
                    contours.size(), contour_array.contours.size(), filtered_count);

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
    // OpenCV 3.2 is more stable with Point2f input for minAreaRect.
    std::vector<cv::Point2f> pts(contour.begin(), contour.end());
    cv::RotatedRect rotated_rect = cv::minAreaRect(pts);
    if (rotated_rect.size.width <= 0.0f || rotated_rect.size.height <= 0.0f)
    {
      return rect;
    }

    // boxPoints in OpenCV 3.2 is happier with a cv::Mat output.
    cv::Mat box_mat;
    cv::boxPoints(rotated_rect, box_mat);

    std::vector<cv::Point2f> box_float;
    box_float.reserve(4);
    for (int i = 0; i < box_mat.rows; ++i)
    {
      box_float.emplace_back(box_mat.at<float>(i, 0), box_mat.at<float>(i, 1));
    }

    // Sort clockwise around the center for a closed rectangle.
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

  cv::drawContours(annotated, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);
  cv::circle(annotated, cv::Point(static_cast<int>(info.center.x), static_cast<int>(info.center.y)),
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
