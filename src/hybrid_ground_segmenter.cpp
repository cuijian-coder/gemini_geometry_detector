#include "gemini_geometry_detector/hybrid_ground_segmenter.h"

#include <cmath>
#include <random>

namespace gemini_geometry_detector
{

void GroundSegmentationParams::Load(const ros::NodeHandle& pnh)
{
  pnh.param("grid_resolution", grid_resolution, grid_resolution);
  pnh.param("height_threshold", height_threshold, height_threshold);
  pnh.param("max_plane_angle", max_plane_angle, max_plane_angle);
  pnh.param("max_height_above_min", max_height_above_min, max_height_above_min);
  pnh.param("min_candidate_points", min_candidate_points, min_candidate_points);
  pnh.param("ransac_distance_threshold", ransac_distance_threshold, ransac_distance_threshold);
  pnh.param("ransac_max_iterations", ransac_max_iterations, ransac_max_iterations);
  pnh.param("min_inlier_ratio", min_inlier_ratio, min_inlier_ratio);

  ROS_INFO("GroundSegmentationParams loaded:");
  ROS_INFO("  grid_resolution: %.3f", grid_resolution);
  ROS_INFO("  height_threshold: %.3f", height_threshold);
  ROS_INFO("  max_plane_angle: %.3f", max_plane_angle);
  ROS_INFO("  max_height_above_min: %.3f", max_height_above_min);
  ROS_INFO("  min_candidate_points: %d", min_candidate_points);
  ROS_INFO("  ransac_distance_threshold: %.3f", ransac_distance_threshold);
  ROS_INFO("  ransac_max_iterations: %d", ransac_max_iterations);
  ROS_INFO("  min_inlier_ratio: %.3f", min_inlier_ratio);
}

GroundResult HybridGroundSegmenter::Run(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input)
{
  GroundResult result;
  result.ground_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
  result.obstacle_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

  auto filtered = preprocessor_.Run(input);
  Segment(*filtered, *result.ground_cloud, *result.obstacle_cloud);
  return result;
}

namespace
{

Eigen::Vector3f ComputeNormal(const Eigen::Vector3f& a,
                              const Eigen::Vector3f& b,
                              const Eigen::Vector3f& c)
{
  Eigen::Vector3f ab = b - a;
  Eigen::Vector3f ac = c - a;
  Eigen::Vector3f n = ab.cross(ac);
  float norm = n.norm();
  if (norm < 1e-6f)
  {
    return Eigen::Vector3f::UnitZ();
  }
  return n / norm;
}

}  // namespace

HybridGroundSegmenter::HybridGroundSegmenter(const GroundSegmentationParams& params)
  : params_(params), preprocessor_(ros::NodeHandle())
{
}

HybridGroundSegmenter::HybridGroundSegmenter(const ros::NodeHandle& pnh)
  : preprocessor_(ros::NodeHandle(pnh, "preprocess"))
{
  params_.Load(pnh);
}

HybridGroundSegmenter::CellIndex HybridGroundSegmenter::GetCellIndex(float x, float y) const
{
  CellIndex idx;
  idx.x = static_cast<int>(std::floor(x / params_.grid_resolution));
  idx.y = static_cast<int>(std::floor(y / params_.grid_resolution));
  return idx;
}

bool HybridGroundSegmenter::IsValidPlane(const Eigen::Vector3f& normal) const
{
  float angle_rad = std::acos(std::abs(normal.z()));
  float angle_deg = angle_rad * 180.0f / static_cast<float>(M_PI);
  return angle_deg <= params_.max_plane_angle;
}

bool HybridGroundSegmenter::FitPlaneRansac(const std::vector<Eigen::Vector3f>& points,
                                            Eigen::Vector3f& best_normal,
                                            float& best_d,
                                            std::vector<size_t>& inlier_indices) const
{
  inlier_indices.clear();
  const size_t num_points = points.size();
  if (num_points < 3)
  {
    return false;
  }

  std::mt19937 rng(0);
  std::uniform_int_distribution<size_t> dist(0, num_points - 1);

  size_t best_inlier_count = 0;
  Eigen::Vector3f best_n = Eigen::Vector3f::UnitZ();
  float best_offset = 0.0f;

  for (int iter = 0; iter < params_.ransac_max_iterations; ++iter)
  {
    size_t i0 = 0, i1 = 0, i2 = 0;
    do
    {
      i0 = dist(rng);
      i1 = dist(rng);
      i2 = dist(rng);
    } while (i0 == i1 || i0 == i2 || i1 == i2);

    Eigen::Vector3f n;
    float d = 0.0f;
    if (!ComputePlaneFromSamples(points, i0, i1, i2, n, d))
    {
      continue;
    }

    const size_t inlier_count = CountInliers(points, n, d, params_.ransac_distance_threshold);
    if (inlier_count > best_inlier_count)
    {
      best_inlier_count = inlier_count;
      best_n = n;
      best_offset = d;
    }
  }

  if (best_inlier_count < static_cast<size_t>(params_.min_candidate_points))
  {
    return false;
  }

  const float inlier_ratio = static_cast<float>(best_inlier_count) /
                             static_cast<float>(num_points);
  if (inlier_ratio < params_.min_inlier_ratio)
  {
    return false;
  }

  // Recompute least-squares plane from inliers for better accuracy.
  inlier_indices.clear();
  for (size_t i = 0; i < points.size(); ++i)
  {
    const float dist_signed = best_n.dot(points[i]) + best_offset;
    if (std::abs(dist_signed) <= params_.ransac_distance_threshold)
    {
      inlier_indices.push_back(i);
    }
  }

  return RefinePlaneFromInliers(points, inlier_indices, best_normal, best_d);
}

bool HybridGroundSegmenter::ComputePlaneFromSamples(const std::vector<Eigen::Vector3f>& points,
                                                     size_t i0,
                                                     size_t i1,
                                                     size_t i2,
                                                     Eigen::Vector3f& normal,
                                                     float& d) const
{
  const auto& p0 = points[i0];
  const auto& p1 = points[i1];
  const auto& p2 = points[i2];

  normal = ComputeNormal(p0, p1, p2);
  if (!IsValidPlane(normal))
  {
    return false;
  }

  d = -(normal.dot(p0));
  return true;
}

size_t HybridGroundSegmenter::CountInliers(const std::vector<Eigen::Vector3f>& points,
                                            const Eigen::Vector3f& normal,
                                            float d,
                                            float threshold) const
{
  size_t inlier_count = 0;
  for (const auto& pt : points)
  {
    const float dist_signed = normal.dot(pt) + d;
    if (std::abs(dist_signed) <= threshold)
    {
      ++inlier_count;
    }
  }
  return inlier_count;
}

bool HybridGroundSegmenter::RefinePlaneFromInliers(const std::vector<Eigen::Vector3f>& points,
                                                    const std::vector<size_t>& inlier_indices,
                                                    Eigen::Vector3f& normal,
                                                    float& d) const
{
  if (inlier_indices.size() < 3)
  {
    return false;
  }

  Eigen::Vector3f mean = Eigen::Vector3f::Zero();
  for (size_t idx : inlier_indices)
  {
    mean += points[idx];
  }
  mean /= static_cast<float>(inlier_indices.size());

  Eigen::Matrix3f scatter = Eigen::Matrix3f::Zero();
  for (size_t idx : inlier_indices)
  {
    const Eigen::Vector3f diff = points[idx] - mean;
    scatter += diff * diff.transpose();
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(scatter);
  normal = solver.eigenvectors().col(0);
  if (normal.z() < 0.0f)
  {
    normal = -normal;
  }
  if (!IsValidPlane(normal))
  {
    return false;
  }

  d = -normal.dot(mean);
  return true;
}

float HybridGroundSegmenter::PointPlaneDistance(const pcl::PointXYZI& pt,
                                                 const Plane& plane) const
{
  return plane.normal.x() * pt.x + plane.normal.y() * pt.y + plane.normal.z() * pt.z + plane.d;
}

void HybridGroundSegmenter::Segment(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                     pcl::PointCloud<pcl::PointXYZI>& ground,
                                     pcl::PointCloud<pcl::PointXYZI>& non_ground)
{
  ground.clear();
  non_ground.clear();

  const auto grid = BuildGrid(cloud);
  const auto cell_planes = EstimateCellPlanes(cloud, grid);
  ClassifyPoints(cloud, cell_planes, ground, non_ground);
}

std::map<HybridGroundSegmenter::CellIndex, HybridGroundSegmenter::CellData>
HybridGroundSegmenter::BuildGrid(const pcl::PointCloud<pcl::PointXYZI>& cloud) const
{
  std::map<CellIndex, CellData> grid;
  for (size_t i = 0; i < cloud.points.size(); ++i)
  {
    const auto& pt = cloud.points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
    {
      continue;
    }
    CellIndex idx = GetCellIndex(pt.x, pt.y);
    auto& cell = grid[idx];
    cell.point_indices.push_back(i);
    if (pt.z < cell.min_z)
    {
      cell.min_z = pt.z;
    }
  }
  return grid;
}

std::vector<Eigen::Vector3f> HybridGroundSegmenter::CollectCandidates(
    const std::map<CellIndex, CellData>& grid,
    const CellIndex& idx,
    const pcl::PointCloud<pcl::PointXYZI>& cloud) const
{
  std::vector<Eigen::Vector3f> candidates;
  for (int dx = -1; dx <= 1; ++dx)
  {
    for (int dy = -1; dy <= 1; ++dy)
    {
      CellIndex nidx{idx.x + dx, idx.y + dy};
      auto it = grid.find(nidx);
      if (it == grid.end())
      {
        continue;
      }
      const auto& ncell = it->second;
      const float threshold = ncell.min_z + params_.max_height_above_min;
      for (size_t pi : ncell.point_indices)
      {
        const auto& pt = cloud.points[pi];
        if (pt.z <= threshold)
        {
          candidates.push_back(Eigen::Vector3f(pt.x, pt.y, pt.z));
        }
      }
    }
  }
  return candidates;
}

std::map<HybridGroundSegmenter::CellIndex, HybridGroundSegmenter::Plane>
HybridGroundSegmenter::EstimateCellPlanes(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                           const std::map<CellIndex, CellData>& grid)
{
  std::map<CellIndex, Plane> cell_planes;

  for (const auto& kv : grid)
  {
    const CellIndex& idx = kv.first;

    const auto candidates = CollectCandidates(grid, idx, cloud);
    if (static_cast<int>(candidates.size()) < params_.min_candidate_points)
    {
      continue;
    }

    Eigen::Vector3f normal;
    float d = 0.0f;
    std::vector<size_t> inliers;
    if (!FitPlaneRansac(candidates, normal, d, inliers))
    {
      continue;
    }

    Plane current;
    current.normal = normal;
    current.d = d;

    cell_planes[idx] = current;
  }

  return cell_planes;
}

void HybridGroundSegmenter::ClassifyPoints(
    const pcl::PointCloud<pcl::PointXYZI>& cloud,
    const std::map<CellIndex, Plane>& cell_planes,
    pcl::PointCloud<pcl::PointXYZI>& ground,
    pcl::PointCloud<pcl::PointXYZI>& non_ground) const
{
  for (size_t i = 0; i < cloud.points.size(); ++i)
  {
    const auto& pt = cloud.points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
    {
      continue;
    }

    const CellIndex idx = GetCellIndex(pt.x, pt.y);
    bool is_ground = false;

    // Check current cell and fallback to neighbours.
    for (int dx = -1; dx <= 1 && !is_ground; ++dx)
    {
      for (int dy = -1; dy <= 1 && !is_ground; ++dy)
      {
        CellIndex nidx{idx.x + dx, idx.y + dy};
        auto it = cell_planes.find(nidx);
        if (it == cell_planes.end())
        {
          continue;
        }
        const float dist = PointPlaneDistance(pt, it->second);
        if (dist >= -params_.height_threshold && dist <= params_.height_threshold)
        {
          is_ground = true;
        }
      }
    }

    if (is_ground)
    {
      ground.push_back(pt);
    }
    else
    {
      non_ground.push_back(pt);
    }
  }
}

}  // namespace gemini_geometry_detector
