#ifndef GEMINI_GEOMETRY_DETECTOR_HYBRID_GROUND_SEGMENTER_H
#define GEMINI_GEOMETRY_DETECTOR_HYBRID_GROUND_SEGMENTER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <map>
#include <memory>
#include <vector>

#include "gemini_geometry_detector/hybird_preprocessor.h"

namespace gemini_geometry_detector
{

struct GroundSegmentationParams
{
  float grid_resolution = 0.20f;
  float height_threshold = 0.03f;
  float max_plane_angle = 20.0f;
  float max_height_above_min = 0.10f;
  int min_candidate_points = 3;
  float ransac_distance_threshold = 0.03f;
  int ransac_max_iterations = 100;
  float min_inlier_ratio = 0.50f;

  void Load(const ros::NodeHandle& pnh);
};

struct GroundResult
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud;

  GroundResult()
      : ground_cloud(new pcl::PointCloud<pcl::PointXYZI>),
        obstacle_cloud(new pcl::PointCloud<pcl::PointXYZI>)
  {
  }
};

class HybridGroundSegmenter
{
public:
  explicit HybridGroundSegmenter(const GroundSegmentationParams& params);
  explicit HybridGroundSegmenter(const ros::NodeHandle& pnh);

  GroundResult Run(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input);

  void Segment(const pcl::PointCloud<pcl::PointXYZI>& cloud,
               pcl::PointCloud<pcl::PointXYZI>& ground,
               pcl::PointCloud<pcl::PointXYZI>& non_ground);

private:
  struct CellIndex
  {
    int x = 0;
    int y = 0;

    bool operator<(const CellIndex& other) const
    {
      if (x != other.x) return x < other.x;
      return y < other.y;
    }
  };

  struct Plane
  {
    Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
    float d = 0.0f;
  };

  struct CellData
  {
    std::vector<size_t> point_indices;
    float min_z = std::numeric_limits<float>::max();
  };

  CellIndex GetCellIndex(float x, float y) const;

  std::map<CellIndex, CellData> BuildGrid(const pcl::PointCloud<pcl::PointXYZI>& cloud) const;

  std::vector<Eigen::Vector3f> CollectCandidates(
      const std::map<CellIndex, CellData>& grid,
      const CellIndex& idx,
      const pcl::PointCloud<pcl::PointXYZI>& cloud) const;

  std::map<CellIndex, Plane> EstimateCellPlanes(
      const pcl::PointCloud<pcl::PointXYZI>& cloud,
      const std::map<CellIndex, CellData>& grid);

  void ClassifyPoints(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                      const std::map<CellIndex, Plane>& cell_planes,
                      pcl::PointCloud<pcl::PointXYZI>& ground,
                      pcl::PointCloud<pcl::PointXYZI>& non_ground) const;

  bool FitPlaneRansac(const std::vector<Eigen::Vector3f>& points,
                      Eigen::Vector3f& best_normal,
                      float& best_d,
                      std::vector<size_t>& inlier_indices) const;

  bool ComputePlaneFromSamples(const std::vector<Eigen::Vector3f>& points,
                               size_t i0,
                               size_t i1,
                               size_t i2,
                               Eigen::Vector3f& normal,
                               float& d) const;

  size_t CountInliers(const std::vector<Eigen::Vector3f>& points,
                      const Eigen::Vector3f& normal,
                      float d,
                      float threshold) const;

  bool RefinePlaneFromInliers(const std::vector<Eigen::Vector3f>& points,
                              const std::vector<size_t>& inlier_indices,
                              Eigen::Vector3f& normal,
                              float& d) const;

  float PointPlaneDistance(const pcl::PointXYZI& pt, const Plane& plane) const;

  bool IsValidPlane(const Eigen::Vector3f& normal) const;

  GroundSegmentationParams params_;
  HybirdPreprocessor preprocessor_;
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_HYBRID_GROUND_SEGMENTER_H
