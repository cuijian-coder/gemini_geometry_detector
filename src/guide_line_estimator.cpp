#include "gemini_geometry_detector/guide_line_estimator.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace gemini_geometry_detector
{

GuideLineEstimator::GuideLineEstimator(const ros::NodeHandle& pnh)
{
  pnh.param("max_contour_points", max_contour_points_, 64);
}

void GuideLineEstimator::setCameraInfo(const sensor_msgs::CameraInfoConstPtr& info)
{
  std::lock_guard<std::mutex> lock(camera_info_mutex_);
  camera_info_ = info;
}

bool GuideLineEstimator::hasCameraInfo() const
{
  std::lock_guard<std::mutex> lock(camera_info_mutex_);
  return camera_info_ != nullptr;
}

bool GuideLineEstimator::estimate(const std::vector<cv::Point>& contour,
                                  const Eigen::Vector3f& ground_normal,
                                  float ground_d,
                                  std::vector<cv::Point>& sampled_contour,
                                  std::vector<Eigen::Vector3f>& guide_line) const
{
  sampled_contour.clear();
  guide_line.clear();

  if (contour.empty())
  {
    return false;
  }

  size_t step = 1;
  if (max_contour_points_ > 0 && contour.size() > static_cast<size_t>(max_contour_points_))
  {
    step = contour.size() / max_contour_points_;
    if (step < 1)
    {
      step = 1;
    }
  }

  for (size_t i = 0; i < contour.size(); i += step)
  {
    Eigen::Vector3f ray;
    if (!pixelToRay(contour[i].x, contour[i].y, ray))
    {
      continue;
    }

    Eigen::Vector3f point;
    if (!rayPlaneIntersection(ray, ground_normal, ground_d, point))
    {
      continue;
    }

    sampled_contour.push_back(contour[i]);
    guide_line.push_back(point);
  }

  return !guide_line.empty();
}

bool GuideLineEstimator::fitRectangleOnPlane(const std::vector<Eigen::Vector3f>& points,
                                             const Eigen::Vector3f& ground_normal,
                                             float ground_d,
                                             std::vector<Eigen::Vector3f>& rectangle) const
{
  rectangle.clear();
  if (points.size() < 2)
  {
    return false;
  }

  Eigen::Vector3f n = ground_normal;
  if (n.norm() < 1e-6f)
  {
    return false;
  }
  n.normalize();

  // Project all points onto the plane and compute centroid.
  std::vector<Eigen::Vector3f> projected;
  projected.reserve(points.size());
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  for (const auto& p : points)
  {
    const float signed_dist = n.dot(p) + ground_d;
    const Eigen::Vector3f proj = p - signed_dist * n;
    projected.push_back(proj);
    centroid += proj;
  }
  centroid /= static_cast<float>(projected.size());

  // Build an arbitrary tangent basis on the plane.
  Eigen::Vector3f arbitrary = Eigen::Vector3f::UnitX();
  if (std::abs(n.x()) > 0.9f)
  {
    arbitrary = Eigen::Vector3f::UnitY();
  }
  Eigen::Vector3f u_axis = arbitrary - n * (n.dot(arbitrary));
  if (u_axis.norm() < 1e-6f)
  {
    return false;
  }
  u_axis.normalize();
  Eigen::Vector3f v_axis = n.cross(u_axis).normalized();

  // Compute covariance in the tangent basis to find principal axes.
  Eigen::Matrix2f scatter = Eigen::Matrix2f::Zero();
  for (const auto& p : projected)
  {
    const Eigen::Vector3f diff = p - centroid;
    const float du = diff.dot(u_axis);
    const float dv = diff.dot(v_axis);
    scatter(0, 0) += du * du;
    scatter(0, 1) += du * dv;
    scatter(1, 0) += du * dv;
    scatter(1, 1) += dv * dv;
  }

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(scatter);
  if (solver.info() != Eigen::Success)
  {
    return false;
  }

  // Principal axes in 3D.
  Eigen::Vector3f axis_u = solver.eigenvectors().col(1).x() * u_axis +
                           solver.eigenvectors().col(1).y() * v_axis;
  Eigen::Vector3f axis_v = n.cross(axis_u).normalized();
  axis_u.normalize();

  // Extents along the principal axes.
  float min_u = std::numeric_limits<float>::max();
  float max_u = std::numeric_limits<float>::lowest();
  float min_v = std::numeric_limits<float>::max();
  float max_v = std::numeric_limits<float>::lowest();
  for (const auto& p : projected)
  {
    const Eigen::Vector3f diff = p - centroid;
    const float u = diff.dot(axis_u);
    const float v = diff.dot(axis_v);
    min_u = std::min(min_u, u);
    max_u = std::max(max_u, u);
    min_v = std::min(min_v, v);
    max_v = std::max(max_v, v);
  }

  const float half_u = 0.5f * (max_u - min_u);
  const float half_v = 0.5f * (max_v - min_v);
  if (half_u < 1e-3f || half_v < 1e-3f)
  {
    return false;
  }

  // Center of the fitted rectangle.
  const Eigen::Vector3f rect_center = centroid + 0.5f * (min_u + max_u) * axis_u +
                                      0.5f * (min_v + max_v) * axis_v;

  // Four corners in clockwise order.
  rectangle.push_back(rect_center + half_u * axis_u + half_v * axis_v);
  rectangle.push_back(rect_center + half_u * axis_u - half_v * axis_v);
  rectangle.push_back(rect_center - half_u * axis_u - half_v * axis_v);
  rectangle.push_back(rect_center - half_u * axis_u + half_v * axis_v);

  return true;
}

bool GuideLineEstimator::pixelToRay(int u, int v, Eigen::Vector3f& ray) const
{
  std::lock_guard<std::mutex> lock(camera_info_mutex_);
  if (!camera_info_)
  {
    return false;
  }

  const auto& k = camera_info_->K;
  const double fx = k[0];
  const double fy = k[4];
  const double cx = k[2];
  const double cy = k[5];

  if (fx <= 0.0 || fy <= 0.0)
  {
    return false;
  }

  ray.x() = static_cast<float>((static_cast<double>(u) - cx) / fx);
  ray.y() = static_cast<float>((static_cast<double>(v) - cy) / fy);
  ray.z() = 1.0f;
  return true;
}

bool GuideLineEstimator::rayPlaneIntersection(const Eigen::Vector3f& ray,
                                              const Eigen::Vector3f& normal,
                                              float d,
                                              Eigen::Vector3f& point) const
{
  const float denom = normal.dot(ray);
  if (std::abs(denom) < 1e-6f)
  {
    return false;
  }

  const float t = -d / denom;
  if (t <= 0.0f)
  {
    return false;
  }

  point = t * ray;
  return std::isfinite(point.x()) && std::isfinite(point.y()) && std::isfinite(point.z());
}

sensor_msgs::PointCloud2 GuideLineEstimator::buildPointCloud2(
    const std_msgs::Header& header,
    const std::string& frame_id,
    const std::vector<Eigen::Vector3f>& points) const
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.reserve(points.size());
  for (const auto& p : points)
  {
    cloud.points.emplace_back(p.x(), p.y(), p.z());
  }
  cloud.width = static_cast<uint32_t>(cloud.points.size());
  cloud.height = 1;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header = header;
  msg.header.frame_id = frame_id;
  return msg;
}

visualization_msgs::Marker GuideLineEstimator::buildLineMarker(
    const std_msgs::Header& header,
    const std::string& frame_id,
    const std::vector<Eigen::Vector3f>& points) const
{
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.header.frame_id = frame_id;
  marker.ns = "guide_line";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.color.r = 1.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  marker.lifetime = ros::Duration(0);

  marker.points.reserve(points.size());
  for (const auto& p : points)
  {
    geometry_msgs::Point pt;
    pt.x = static_cast<double>(p.x());
    pt.y = static_cast<double>(p.y());
    pt.z = static_cast<double>(p.z());
    marker.points.push_back(pt);
  }

  return marker;
}

Contour3DArray GuideLineEstimator::buildContour3DArray(
    const std_msgs::Header& header,
    const std::vector<cv::Point>& contour_2d,
    const std::vector<Eigen::Vector3f>& points_3d,
    double area) const
{
  Contour3DArray array;
  array.header = header;

  Contour3DInfo info;
  info.id = 0;
  info.area = area;

  // 2D centroid and bounding box from the sampled contour.
  cv::Moments m = cv::moments(contour_2d);
  cv::Point2f center(0, 0);
  if (m.m00 > 0)
  {
    center.x = static_cast<float>(m.m10 / m.m00);
    center.y = static_cast<float>(m.m01 / m.m00);
  }
  cv::Rect bbox = cv::boundingRect(contour_2d);

  info.center_2d.x = center.x;
  info.center_2d.y = center.y;
  info.center_2d.z = 0.0;

  info.bbox_tl.x = bbox.x;
  info.bbox_tl.y = bbox.y;
  info.bbox_tl.z = 0.0;
  info.bbox_br.x = bbox.x + bbox.width;
  info.bbox_br.y = bbox.y + bbox.height;
  info.bbox_br.z = 0.0;

  // 3D centroid and depth statistics.
  Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
  float min_z = std::numeric_limits<float>::max();
  float max_z = std::numeric_limits<float>::lowest();
  float sum_z = 0.0f;

  info.points_2d.reserve(contour_2d.size());
  info.points_3d.reserve(points_3d.size());

  for (size_t i = 0; i < contour_2d.size(); ++i)
  {
    geometry_msgs::Point32 p2;
    p2.x = contour_2d[i].x;
    p2.y = contour_2d[i].y;
    p2.z = 0.0f;
    info.points_2d.push_back(p2);

    if (i < points_3d.size())
    {
      const auto& p = points_3d[i];
      geometry_msgs::Point p3;
      p3.x = static_cast<double>(p.x());
      p3.y = static_cast<double>(p.y());
      p3.z = static_cast<double>(p.z());
      info.points_3d.push_back(p3);

      centroid += p;
      min_z = std::min(min_z, p.z());
      max_z = std::max(max_z, p.z());
      sum_z += p.z();
    }
  }

  if (!points_3d.empty())
  {
    centroid /= static_cast<float>(points_3d.size());
    info.center_3d.x = static_cast<double>(centroid.x());
    info.center_3d.y = static_cast<double>(centroid.y());
    info.center_3d.z = static_cast<double>(centroid.z());
    info.mean_depth = static_cast<double>(sum_z / static_cast<float>(points_3d.size()));
    info.min_depth = static_cast<double>(min_z);
    info.max_depth = static_cast<double>(max_z);
  }
  else
  {
    info.center_3d.x = info.center_3d.y = info.center_3d.z = 0.0;
    info.mean_depth = info.min_depth = info.max_depth = 0.0;
  }

  array.contours.push_back(info);
  return array;
}

}  // namespace gemini_geometry_detector
