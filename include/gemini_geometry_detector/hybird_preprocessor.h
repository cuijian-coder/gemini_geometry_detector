#ifndef GEMINI_GEOMETRY_DETECTOR_HYBRID_PREPROCESSOR_H
#define GEMINI_GEOMETRY_DETECTOR_HYBRID_PREPROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <map>
#include <memory>
#include <vector>

namespace gemini_geometry_detector
{

struct PreprocessParams
{
  float roi_x_min = -20.0f;
  float roi_x_max = 0.001f;
  float roi_y_min = -8.0f;
  float roi_y_max = 8.0f;
  float roi_z_min = -2.0f;
  float roi_z_max = 2.0f;
  float voxel_leaf = 0.05f;
  int sor_mean_k = 30;
  float sor_stddev = 1.0f;

  void Load(const ros::NodeHandle& pnh);
};

class HybirdPreprocessor {
 public:
  explicit HybirdPreprocessor(const ros::NodeHandle& pnh);

  pcl::PointCloud<pcl::PointXYZI>::Ptr Run(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& input);

 private:
  void RemoveNaNAndZero(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  void ApplyRoi(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  void ApplyVoxel(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  void ApplySor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

  PreprocessParams params_;
};

}  // namespace gemini_geometry_detector

#endif  
