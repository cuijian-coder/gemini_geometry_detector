#include "gemini_geometry_detector/color_region_detector.h"
#include "gemini_geometry_detector/ContourArray.h"
#include "gemini_geometry_detector/ContourInfo.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

namespace gemini_geometry_detector
{

ColorRegionDetector::ColorRegionDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), it_(nh), has_image_(false)
{
  loadParameters();

  image_sub_ = it_.subscribe(input_topic_, 1, &ColorRegionDetector::imageCallback, this);
  mask_pub_ = it_.advertise("color/mask", 1);
  annotated_pub_ = it_.advertise("color/annotated", 1);
  contours_pub_ = nh_.advertise<ContourArray>("color/contours", 1);

  process_timer_ = nh_.createTimer(ros::Duration(1.0 / process_rate_), &ColorRegionDetector::timerCallback, this);

  ROS_INFO("ColorRegionDetector started.");
  ROS_INFO("Input topic: %s", input_topic_.c_str());
  ROS_INFO("HSV range: H[%d,%d] S[%d,%d] V[%d,%d]", h_min_, h_max_, s_min_, s_max_, v_min_, v_max_);
  ROS_INFO("Process rate: %.1f Hz", process_rate_);
}

void ColorRegionDetector::loadParameters()
{
  pnh_.param<std::string>("input_topic", input_topic_, "/camera/color/image_raw");

  pnh_.param("h_min", h_min_, 20);
  pnh_.param("h_max", h_max_, 40);
  pnh_.param("s_min", s_min_, 80);
  pnh_.param("s_max", s_max_, 255);
  pnh_.param("v_min", v_min_, 80);
  pnh_.param("v_max", v_max_, 255);

  pnh_.param("morph_kernel_size", morph_kernel_size_, 5);
  pnh_.param("min_contour_area", min_contour_area_, 200);
  pnh_.param("max_contour_points", max_contour_points_, 64);
  pnh_.param("process_rate", process_rate_, 10.0);

  // Validate kernel size: must be positive odd
  if (morph_kernel_size_ > 0 && morph_kernel_size_ % 2 == 0)
  {
    morph_kernel_size_ += 1;
    ROS_WARN("morph_kernel_size must be odd, adjusted to %d", morph_kernel_size_);
  }
}

void ColorRegionDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(image_mutex_);
  latest_image_ = msg;
  has_image_ = true;
}

void ColorRegionDetector::timerCallback(const ros::TimerEvent& event)
{
  processLatestImage();
}

void ColorRegionDetector::processLatestImage()
{
  sensor_msgs::ImageConstPtr image_msg;
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!has_image_)
    {
      return;
    }
    image_msg = latest_image_;
  }

  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv::Mat hsv;
  cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), mask);

  if (morph_kernel_size_ > 0)
  {
    cv::Mat element = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(morph_kernel_size_, morph_kernel_size_));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);
  }

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  // Annotated image
  cv::Mat annotated = cv_ptr->image.clone();

  ContourArray contour_array;
  contour_array.header = image_msg->header;

  int id = 0;
  for (const auto& contour : contours)
  {
    double area = cv::contourArea(contour);
    if (area < min_contour_area_)
    {
      continue;
    }

    cv::Moments m = cv::moments(contour);
    cv::Point2f center;
    if (m.m00 > 0)
    {
      center.x = static_cast<float>(m.m10 / m.m00);
      center.y = static_cast<float>(m.m01 / m.m00);
    }
    else
    {
      center = cv::Point2f(0, 0);
    }

    cv::Rect bbox = cv::boundingRect(contour);

    // Draw on annotated image
    cv::drawContours(annotated, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);
    cv::circle(annotated, center, 4, cv::Scalar(0, 0, 255), -1);
    cv::rectangle(annotated, bbox, cv::Scalar(255, 0, 0), 2);

    // Fill message
    ContourInfo info;
    info.id = id++;
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

    // Downsample contour points if needed
    size_t step = 1;
    if (max_contour_points_ > 0 && contour.size() > static_cast<size_t>(max_contour_points_))
    {
      step = contour.size() / max_contour_points_;
      if (step < 1) step = 1;
    }
    for (size_t i = 0; i < contour.size(); i += step)
    {
      geometry_msgs::Point32 p;
      p.x = contour[i].x;
      p.y = contour[i].y;
      p.z = 0.0f;
      info.points.push_back(p);
    }

    contour_array.contours.push_back(info);
  }

  // Publish mask
  cv_bridge::CvImage mask_msg;
  mask_msg.header = image_msg->header;
  mask_msg.encoding = sensor_msgs::image_encodings::MONO8;
  mask_msg.image = mask;
  mask_pub_.publish(mask_msg.toImageMsg());

  // Publish annotated image
  cv_bridge::CvImage annotated_msg;
  annotated_msg.header = image_msg->header;
  annotated_msg.encoding = sensor_msgs::image_encodings::BGR8;
  annotated_msg.image = annotated;
  annotated_pub_.publish(annotated_msg.toImageMsg());

  // Publish contours
  contours_pub_.publish(contour_array);
}

}  // namespace gemini_geometry_detector
