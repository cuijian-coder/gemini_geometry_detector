#include "gemini_geometry_detector/color_region_detector.h"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cmath>

namespace gemini_geometry_detector
{

ColorRegionDetector::ColorRegionDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), it_(nh),
    ground_segmenter_(pnh_.resolveName("ground_segmentation", true)),
    first_sync_received_(false)
{
  loadParameters();

  depth_projector_.setDepthScale(depth_scale_);

  mask_pub_ = it_.advertise("color/mask", 1);
  annotated_pub_ = it_.advertise("color/annotated", 1);
  contours_pub_ = nh_.advertise<ContourArray>("color/contours", 1);
  ground_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("color/ground_cloud", 1);
  depth_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("color/depth_cloud", 1);

  rgb_sub_filter_.subscribe(it_, input_topic_, 1);

  if (use_point_cloud_)
  {
    cloud_sub_.subscribe(nh_, point_cloud_topic_, 1);
    cloud_sync_.reset(new message_filters::Synchronizer<CloudSyncPolicy>(
        CloudSyncPolicy(10), rgb_sub_filter_, cloud_sub_));
    cloud_sync_->registerCallback(boost::bind(&ColorRegionDetector::rgbCloudCallback, this, _1, _2));
  }
  else
  {
    depth_sub_.subscribe(nh_, depth_topic_, 1);
    info_sub_.subscribe(nh_, camera_info_topic_, 1);

    sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
        SyncPolicy(10), rgb_sub_filter_, depth_sub_, info_sub_));
    sync_->registerCallback(boost::bind(&ColorRegionDetector::rgbDepthInfoCallback, this, _1, _2, _3));
  }

  ROS_INFO("ColorRegionDetector started.");
  ROS_INFO("RGB topic: %s", input_topic_.c_str());
  if (use_point_cloud_)
  {
    ROS_INFO("PointCloud mode enabled.");
    ROS_INFO("Point cloud topic: %s", point_cloud_topic_.c_str());
  }
  else
  {
    ROS_INFO("Depth topic: %s", depth_topic_.c_str());
    ROS_INFO("CameraInfo topic: %s", camera_info_topic_.c_str());
  }
  ROS_INFO("HSV range: H[%d,%d] S[%d,%d] V[%d,%d]", h_min_, h_max_, s_min_, s_max_, v_min_, v_max_);
}

void ColorRegionDetector::loadParameters()
{
  pnh_.param<std::string>("input_topic", input_topic_, "/camera/color/image_raw");
  pnh_.param<std::string>("depth_topic", depth_topic_, "/camera/depth/image_raw");
  pnh_.param<std::string>("camera_info_topic", camera_info_topic_, "/camera/color/camera_info");
  pnh_.param<std::string>("point_cloud_topic", point_cloud_topic_, "/camera/depth_registered/points");
  pnh_.param<bool>("use_point_cloud", use_point_cloud_, false);

  pnh_.param("h_min", h_min_, 20);
  pnh_.param("h_max", h_max_, 40);
  pnh_.param("s_min", s_min_, 80);
  pnh_.param("s_max", s_max_, 255);
  pnh_.param("v_min", v_min_, 80);
  pnh_.param("v_max", v_max_, 255);

  pnh_.param("morph_kernel_size", morph_kernel_size_, 5);
  pnh_.param("min_contour_area", min_contour_area_, 200);
  pnh_.param("max_contour_points", max_contour_points_, 64);
  pnh_.param("depth_scale", depth_scale_, 0.001);

  if (morph_kernel_size_ > 0 && morph_kernel_size_ % 2 == 0)
  {
    morph_kernel_size_ += 1;
    ROS_WARN("morph_kernel_size must be odd, adjusted to %d", morph_kernel_size_);
  }
}

void ColorRegionDetector::rgbDepthInfoCallback(const sensor_msgs::ImageConstPtr& rgb_msg,
                                               const sensor_msgs::ImageConstPtr& depth_msg,
                                               const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  if (!first_sync_received_)
  {
    ROS_INFO("First synced frame received: RGB %dx%d, Depth %dx%d, CameraInfo frame_id: %s",
             rgb_msg->width, rgb_msg->height,
             depth_msg->width, depth_msg->height,
             info_msg->header.frame_id.c_str());
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

  processFrame(rgb_msg, cv_rgb->image, depth_msg, info_msg);
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

  processFrameCloud(rgb_msg, cv_rgb->image, cloud_msg);
}

void ColorRegionDetector::processFrame(const sensor_msgs::ImageConstPtr& rgb_msg,
                                       const cv::Mat& bgr_image,
                                       const sensor_msgs::ImageConstPtr& depth_msg,
                                       const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  cv::Mat mask = createColorMask(bgr_image);
  applyMorphology(mask);

  std::vector<std::vector<cv::Point>> contours;
  detectContours(mask, contours);

  cv::Mat annotated = bgr_image.clone();
  ContourArray contour_array;
  contour_array.header = rgb_msg->header;

  int id = 0;
  int filtered_count = 0;

  for (const auto& contour : contours)
  {
    ContourInfo info;
    if (!buildContourInfo(contour, id, info))
    {
      ++filtered_count;
      continue;
    }

    drawContourOnImage(annotated, contour, info);
    contour_array.contours.push_back(info);
    ++id;
  }

  // Project depth to 3D point cloud
  depth_projector_.setCameraInfo(info_msg);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  if (depth_projector_.convertToPointCloud(depth_msg, cloud))
  {
    // Segment ground (preprocessing is done inside HybridGroundSegmenter)
    auto result = ground_segmenter_.Run(cloud);

    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*result.ground_cloud, ground_msg);
    ground_msg.header = rgb_msg->header;
    ground_msg.header.frame_id = depth_msg->header.frame_id;
    ground_cloud_pub_.publish(ground_msg);

    sensor_msgs::PointCloud2 depth_cloud_msg;
    pcl::toROSMsg(*cloud, depth_cloud_msg);
    depth_cloud_msg.header = rgb_msg->header;
    depth_cloud_msg.header.frame_id = depth_msg->header.frame_id;
    depth_cloud_pub_.publish(depth_cloud_msg);

    ROS_INFO_THROTTLE(1.0,
                      "Image %dx%d, raw contours: %zu, valid 2D: %zu, filtered: %d, "
                      "depth cloud: %zu, ground: %zu",
                      bgr_image.cols, bgr_image.rows,
                      contours.size(), contour_array.contours.size(), filtered_count,
                      cloud->size(), result.ground_cloud->size());
  }
  else
  {
    ROS_WARN_THROTTLE(1.0, "Failed to convert depth to point cloud");
  }

  publishResults(rgb_msg->header, mask, annotated, contour_array);
}

void ColorRegionDetector::processFrameCloud(const sensor_msgs::ImageConstPtr& rgb_msg,
                                            const cv::Mat& bgr_image,
                                            const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  cv::Mat mask = createColorMask(bgr_image);
  applyMorphology(mask);

  std::vector<std::vector<cv::Point>> contours;
  detectContours(mask, contours);

  cv::Mat annotated = bgr_image.clone();
  ContourArray contour_array;
  contour_array.header = rgb_msg->header;

  int id = 0;
  int filtered_count = 0;

  for (const auto& contour : contours)
  {
    ContourInfo info;
    if (!buildContourInfo3DFromCloud(contour, id, info, cloud_msg))
    {
      ++filtered_count;
      continue;
    }

    drawContourOnImage(annotated, contour, info);
    contour_array.contours.push_back(info);
    ++id;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  auto result = ground_segmenter_.Run(cloud);

  sensor_msgs::PointCloud2 ground_msg;
  pcl::toROSMsg(*result.ground_cloud, ground_msg);
  ground_msg.header = rgb_msg->header;
  ground_msg.header.frame_id = cloud_msg->header.frame_id;
  ground_cloud_pub_.publish(ground_msg);

  depth_cloud_pub_.publish(*cloud_msg);

  ROS_INFO_THROTTLE(1.0,
                    "Image %dx%d, raw contours: %zu, valid 2D: %zu, filtered: %d, "
                    "cloud: %zu, ground: %zu",
                    bgr_image.cols, bgr_image.rows,
                    contours.size(), contour_array.contours.size(), filtered_count,
                    cloud->size(), result.ground_cloud->size());

  publishResults(rgb_msg->header, mask, annotated, contour_array);
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
                                           ContourInfo& info)
{
  double area = cv::contourArea(contour);
  if (area < min_contour_area_)
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

bool ColorRegionDetector::buildContourInfo3DFromCloud(const std::vector<cv::Point>& contour,
                                                      int id,
                                                      ContourInfo& info,
                                                      const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  if (!buildContourInfo(contour, id, info))
  {
    return false;
  }

  // If the cloud is unorganized, we cannot do a per-pixel lookup.
  if (cloud_msg->height <= 1 || cloud_msg->width == 0)
  {
    return true;
  }

  const int u = static_cast<int>(std::round(info.center.x));
  const int v = static_cast<int>(std::round(info.center.y));
  if (u < 0 || u >= static_cast<int>(cloud_msg->width) ||
      v < 0 || v >= static_cast<int>(cloud_msg->height))
  {
    return false;
  }

  try
  {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

    const size_t idx = static_cast<size_t>(v) * cloud_msg->width + static_cast<size_t>(u);
    iter_x += idx;
    iter_y += idx;
    iter_z += idx;

    const float x = *iter_x;
    const float y = *iter_y;
    const float z = *iter_z;
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
    {
      return false;
    }

    // Store the center depth as a hint in the otherwise 2D contour info.
    info.center.z = z;
  }
  catch (const std::exception& e)
  {
    ROS_WARN_THROTTLE(1.0, "Failed to lookup 3D point from cloud: %s", e.what());
    return false;
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
