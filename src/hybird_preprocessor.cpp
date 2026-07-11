#include "gemini_geometry_detector/hybird_preprocessor.h"

#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <ros/ros.h>

namespace gemini_geometry_detector
{

void PreprocessParams::Load(const ros::NodeHandle& pnh)
{
  pnh.param<float>("preprocess/roi_x_min", roi_x_min, roi_x_min);
  pnh.param<float>("preprocess/roi_x_max", roi_x_max, roi_x_max);
  pnh.param<float>("preprocess/roi_y_min", roi_y_min, roi_y_min);
  pnh.param<float>("preprocess/roi_y_max", roi_y_max, roi_y_max);
  pnh.param<float>("preprocess/roi_z_min", roi_z_min, roi_z_min);
  pnh.param<float>("preprocess/roi_z_max", roi_z_max, roi_z_max);
  pnh.param<float>("preprocess/voxel_leaf", voxel_leaf, voxel_leaf);
  pnh.param<int>("preprocess/sor_mean_k", sor_mean_k, sor_mean_k);
  pnh.param<float>("preprocess/sor_stddev", sor_stddev, sor_stddev);
}

HybirdPreprocessor::HybirdPreprocessor(const ros::NodeHandle& pnh)
{
  params_.Load(pnh);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr HybirdPreprocessor::Run(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input) {
  auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>(*input));

  const std::size_t input_size = cloud->size();
  RemoveNaNAndZero(cloud);
  const std::size_t nan_size = cloud->size();
  ApplyRoi(cloud);
  const std::size_t roi_size = cloud->size();
  ApplyVoxel(cloud);
  const std::size_t voxel_size = cloud->size();
  ApplySor(cloud);
  const std::size_t sor_size = cloud->size();

  ROS_INFO("[HybirdPreprocessor] input %zu -> NaN/Zero %zu -> ROI %zu -> Voxel %zu -> SOR %zu",
           input_size, nan_size, roi_size, voxel_size, sor_size);

  return cloud;
}

void HybirdPreprocessor::RemoveNaNAndZero(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  pcl::PointCloud<pcl::PointXYZI> filtered;
  filtered.reserve(cloud->size());
  for (const auto& pt : cloud->points) {
    if (pt.x != 0.0f || pt.y != 0.0f || pt.z != 0.0f) {
      filtered.push_back(pt);
    }
  }
  *cloud = filtered;
}

void HybirdPreprocessor::ApplyRoi(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  pcl::CropBox<pcl::PointXYZI> crop;
  crop.setInputCloud(cloud);
  crop.setMin(Eigen::Vector4f(params_.roi_x_min, params_.roi_y_min, params_.roi_z_min, 1.0f));
  crop.setMax(Eigen::Vector4f(params_.roi_x_max, params_.roi_y_max, params_.roi_z_max, 1.0f));
  crop.filter(*cloud);
}

void HybirdPreprocessor::ApplyVoxel(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  pcl::VoxelGrid<pcl::PointXYZI> voxel;
  voxel.setInputCloud(cloud);
  voxel.setLeafSize(params_.voxel_leaf, params_.voxel_leaf, params_.voxel_leaf);
  voxel.filter(*cloud);
}

void HybirdPreprocessor::ApplySor(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
  pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(params_.sor_mean_k);
  sor.setStddevMulThresh(params_.sor_stddev);
  sor.filter(*cloud);
}

}  
