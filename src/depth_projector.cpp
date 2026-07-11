#include "gemini_geometry_detector/depth_projector.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace gemini_geometry_detector
{

DepthProjector::DepthProjector()
  : fx_(0.0), fy_(0.0), cx_(0.0), cy_(0.0),
    depth_scale_(0.001), has_camera_info_(false)
{
}

void DepthProjector::setDepthScale(double depth_scale)
{
  depth_scale_ = depth_scale;
}

void DepthProjector::setCameraInfo(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  if (!camera_info)
  {
    ROS_WARN("DepthProjector received null CameraInfo");
    has_camera_info_ = false;
    return;
  }

  fx_ = camera_info->K[0];
  fy_ = camera_info->K[4];
  cx_ = camera_info->K[2];
  cy_ = camera_info->K[5];

  if (fx_ <= 0.0 || fy_ <= 0.0)
  {
    ROS_ERROR("Invalid camera intrinsics: fx=%.3f, fy=%.3f", fx_, fy_);
    has_camera_info_ = false;
    return;
  }

  has_camera_info_ = true;
}

bool DepthProjector::convertDepthValue(uint16_t raw, float& depth_m) const
{
  if (raw == 0)
  {
    return false;
  }
  depth_m = static_cast<float>(raw) * depth_scale_;
  return std::isfinite(depth_m);
}

bool DepthProjector::convertToPointCloud(const sensor_msgs::ImageConstPtr& depth_msg,
                                          pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const
{
  if (!has_camera_info_)
  {
    ROS_ERROR_THROTTLE(5.0, "DepthProjector: camera info not set");
    return false;
  }

  if (!cloud)
  {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
  }
  cloud->clear();
  cloud->header.frame_id = depth_msg->header.frame_id;
  cloud->width = depth_msg->width;
  cloud->height = depth_msg->height;
  cloud->is_dense = false;

  cv_bridge::CvImageConstPtr cv_depth;
  try
  {
    cv_depth = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("DepthProjector cv_bridge exception: %s", e.what());
    return false;
  }

  const cv::Mat& depth_image = cv_depth->image;
  cloud->reserve(depth_image.rows * depth_image.cols);

  for (int v = 0; v < depth_image.rows; ++v)
  {
    for (int u = 0; u < depth_image.cols; ++u)
    {
      float depth_m = 0.0f;
      if (!convertDepthValue(depth_image.at<uint16_t>(v, u), depth_m))
      {
        continue;
      }

      pcl::PointXYZI pt;
      pt.z = depth_m;
      pt.x = static_cast<float>((static_cast<double>(u) - cx_) * depth_m / fx_);
      pt.y = static_cast<float>((static_cast<double>(v) - cy_) * depth_m / fy_);
      pt.intensity = depth_m;
      cloud->push_back(pt);
    }
  }

  return true;
}

bool DepthProjector::convertToPointCloud(const sensor_msgs::ImageConstPtr& depth_msg,
                                          sensor_msgs::PointCloud2& cloud_msg,
                                          const std::string& frame_id) const
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  if (!convertToPointCloud(depth_msg, cloud))
  {
    return false;
  }

  cloud_msg.header = depth_msg->header;
  cloud_msg.header.frame_id = frame_id.empty() ? depth_msg->header.frame_id : frame_id;
  cloud_msg.height = 1;
  cloud_msg.width = cloud->size();
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");

  for (const auto& pt : cloud->points)
  {
    *iter_x = pt.x;
    *iter_y = pt.y;
    *iter_z = pt.z;
    *iter_r = 255;
    *iter_g = 0;
    *iter_b = 0;
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
  }

  return true;
}

}  // namespace gemini_geometry_detector
