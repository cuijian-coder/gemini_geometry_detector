#ifndef GEMINI_GEOMETRY_DETECTOR_DEPTH_PROJECTOR_H
#define GEMINI_GEOMETRY_DETECTOR_DEPTH_PROJECTOR_H

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace gemini_geometry_detector
{

class DepthProjector
{
public:
  DepthProjector();

  void setDepthScale(double depth_scale);
  void setCameraInfo(const sensor_msgs::CameraInfoConstPtr& camera_info);

  bool convertToPointCloud(const sensor_msgs::ImageConstPtr& depth_msg,
                           pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const;

  bool convertToPointCloud(const sensor_msgs::ImageConstPtr& depth_msg,
                           sensor_msgs::PointCloud2& cloud_msg,
                           const std::string& frame_id) const;

private:
  bool convertDepthValue(uint16_t raw, float& depth_m) const;

  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double depth_scale_;
  bool has_camera_info_;
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_DEPTH_PROJECTOR_H
