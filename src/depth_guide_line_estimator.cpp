#include "gemini_geometry_detector/depth_guide_line_estimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <ros/ros.h>

namespace gemini_geometry_detector
{

void DepthGuideLineEstimator::configure(const CameraIntrinsics& intrinsics,
                                        double image_scale,
                                        double target_angle,
                                        double /*roi_y_ratio*/,
                                        int /*roi_y*/,
                                        double look_ahead_m)
{
  camera_intrinsics_.fx = intrinsics.fx * image_scale;
  camera_intrinsics_.fy = intrinsics.fy * image_scale;
  camera_intrinsics_.cx = intrinsics.cx * image_scale;
  camera_intrinsics_.cy = intrinsics.cy * image_scale;

  image_scale_ = image_scale;
  target_angle_ = target_angle;
  look_ahead_m_ = look_ahead_m;

  ROS_INFO("[DepthGuideLineEstimator] configured:");
  ROS_INFO("  scaled intrinsics: fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
           camera_intrinsics_.fx, camera_intrinsics_.fy,
           camera_intrinsics_.cx, camera_intrinsics_.cy);
  ROS_INFO("  image_scale:       %.2f", image_scale_);
  ROS_INFO("  target_angle:      %.4f rad (%.1f deg)",
           target_angle_, target_angle_ * 180.0 / M_PI);
  ROS_INFO("  look_ahead_m:      %.3f m", look_ahead_m_);
}

void DepthGuideLineEstimator::setGroundPlane(const Eigen::Vector3f& normal, float d)
{
  ground_normal_ = normal;
  if (ground_normal_.norm() > 1e-6f)
  {
    ground_normal_.normalize();
  }
  ground_d_ = d;

  // Ensure a consistent sign convention: the camera origin should be on the
  // side pointed to by the ground normal (i.e. d > 0).  If d < 0, the normal
  // currently points toward the ground rather than away from it; flipping both
  // n and d keeps the same geometric plane but makes the ray-plane intersection
  // t = -d / (n . ray) positive for rays heading toward the ground.
  if (ground_d_ < 0.0f)
  {
    ground_normal_ = -ground_normal_;
    ground_d_ = -ground_d_;
  }

  ground_plane_received_ = true;

  static bool first = true;
  if (first)
  {
    ROS_INFO("[DepthGuideLineEstimator] first ground plane set: "
             "n=(%.3f, %.3f, %.3f), d=%.3f",
             ground_normal_.x(), ground_normal_.y(),
             ground_normal_.z(), ground_d_);
    first = false;
  }

  ROS_INFO_THROTTLE(5.0,
                    "[DepthGuideLineEstimator] ground plane: "
                    "n=(%.3f, %.3f, %.3f), d=%.3f",
                    ground_normal_.x(), ground_normal_.y(),
                    ground_normal_.z(), ground_d_);
}

GuideLineError DepthGuideLineEstimator::estimate(const std::vector<cv::Point>& largest_contour,
                                                 const std::vector<cv::Point>& rect_contour,
                                                 const cv::Size& image_size,
                                                 const std_msgs::Header& header,
                                                 cv::Mat& annotated)
{
  GuideLineError error_msg;
  error_msg.header = header;
  error_msg.valid = false;
  error_msg.yaw_error = 0.0;
  error_msg.lateral_error_n = 0.0;
  error_msg.lateral_error_m = 0.0;
  error_msg.roi_point.x = 0.0;
  error_msg.roi_point.y = 0.0;
  error_msg.roi_point.z = 0.0;

  if (largest_contour.empty())
  {
    return error_msg;
  }

  if (!ground_plane_received_)
  {
    ROS_WARN_THROTTLE(1.0, "DepthGuideLineEstimator: ground plane not received yet");
    return error_msg;
  }

  // Draw contour/rectangle for visualization.
  if (!rect_contour.empty())
  {
    cv::polylines(annotated, std::vector<std::vector<cv::Point>>{rect_contour},
                  true, cv::Scalar(0, 255, 255), 2);
  }
  cv::polylines(annotated, std::vector<std::vector<cv::Point>>{largest_contour},
                true, cv::Scalar(255, 0, 0), 2);

  // Back-project contour pixels to ground-plane points.
  // For a stationary camera, using all points gives a much more stable line
  // fit than the previous fixed-size uniform sampling.  We only subsample when
  // the contour is larger than max_contour_points_, and in that case use a
  // fixed step so the selected point set is deterministic.
  std::vector<Eigen::Vector3f> ground_points;
  ground_points.reserve(std::min(max_contour_points_,
                                 static_cast<int>(largest_contour.size())));

  const size_t contour_size = largest_contour.size();
  const size_t step = contour_size <= static_cast<size_t>(max_contour_points_)
                          ? 1
                          : (contour_size + max_contour_points_ - 1) /
                                static_cast<size_t>(max_contour_points_);

  int skipped_rays = 0;
  int skipped_intersections = 0;
  for (size_t i = 0; i < largest_contour.size(); i += step)
  {
    Eigen::Vector3f ray;
    if (!pixelToRay(largest_contour[i].x, largest_contour[i].y, ray))
    {
      ++skipped_rays;
      continue;
    }

    Eigen::Vector3f point;
    if (!rayPlaneIntersection(ray, ground_normal_, ground_d_, point))
    {
      ++skipped_intersections;
      continue;
    }

    ground_points.push_back(point);
  }

  if (ground_points.size() < 2)
  {
    ROS_WARN_THROTTLE(1.0,
                      "[DepthGuideLineEstimator] not enough ground-plane points: "
                      "%zu valid (contour_pts=%zu, skipped_rays=%d, "
                      "skipped_intersections=%d). "
                      "Ground plane: n=(%.3f, %.3f, %.3f), d=%.3f. "
                      "Check ground-plane sign convention / frame_id.",
                      ground_points.size(), largest_contour.size(),
                      skipped_rays, skipped_intersections,
                      ground_normal_.x(), ground_normal_.y(),
                      ground_normal_.z(), ground_d_);
    return error_msg;
  }

  Eigen::Vector3f ground_point;
  Eigen::Vector3f ground_direction;
  if (!fitGroundLine(ground_points, ground_point, ground_direction))
  {
    ROS_WARN_THROTTLE(1.0,
                      "[DepthGuideLineEstimator] failed to fit ground line "
                      "from %zu points", ground_points.size());
    return error_msg;
  }

  const double yaw_error = computeYawError(ground_direction);
  const double lateral_error_m = computeLateralErrorM(ground_point, ground_direction);

  error_msg.valid = true;
  error_msg.yaw_error = yaw_error;
  error_msg.lateral_error_n = 0.0;  // Not used by depth estimator.
  error_msg.lateral_error_m = lateral_error_m;

  // Report a reference ROI point projected onto the image plane for compatibility.
  const Eigen::Vector3f cam_z = Eigen::Vector3f::UnitZ();
  Eigen::Vector3f forward = cam_z - cam_z.dot(ground_normal_) * ground_normal_;
  if (forward.norm() < 1e-6f)
  {
    const Eigen::Vector3f cam_y = Eigen::Vector3f::UnitY();
    forward = cam_y - cam_y.dot(ground_normal_) * ground_normal_;
  }
  forward.normalize();

  const Eigen::Vector3f ref_point = (-ground_d_ * ground_normal_) + static_cast<float>(look_ahead_m_) * forward;
  if (std::fabs(ref_point.z()) > 1e-6f)
  {
    error_msg.roi_point.x = ref_point.x() / ref_point.z();
    error_msg.roi_point.y = ref_point.y() / ref_point.z();
    error_msg.roi_point.z = ref_point.z();
  }

  ROS_INFO_THROTTLE(1.0,
                    "[DepthGuideLineEstimator] result: contour_pts=%zu, "
                    "ground_pts=%zu, skipped_rays=%d, skipped_intersections=%d, "
                    "yaw=%.4f rad (%.1f deg), lateral=%.4f m, look_ahead=%.3f m",
                    largest_contour.size(), ground_points.size(),
                    skipped_rays, skipped_intersections,
                    yaw_error, yaw_error * 180.0 / M_PI,
                    lateral_error_m, look_ahead_m_);

  return error_msg;
}

bool DepthGuideLineEstimator::pixelToRay(int u, int v, Eigen::Vector3f& ray) const
{
  if (camera_intrinsics_.fx <= 0.0 || camera_intrinsics_.fy <= 0.0)
  {
    return false;
  }

  ray.x() = (static_cast<float>(u) - static_cast<float>(camera_intrinsics_.cx)) /
            static_cast<float>(camera_intrinsics_.fx);
  ray.y() = (static_cast<float>(v) - static_cast<float>(camera_intrinsics_.cy)) /
            static_cast<float>(camera_intrinsics_.fy);
  ray.z() = 1.0f;
  ray.normalize();
  return true;
}

bool DepthGuideLineEstimator::rayPlaneIntersection(const Eigen::Vector3f& ray,
                                                   const Eigen::Vector3f& normal,
                                                   float d,
                                                   Eigen::Vector3f& point) const
{
  const float denom = normal.dot(ray);
  if (std::fabs(denom) < 1e-6f)
  {
    return false;
  }

  const float t = -d / denom;
  if (t <= 0.0f)
  {
    return false;
  }

  point = t * ray;
  return true;
}

bool DepthGuideLineEstimator::fitGroundLine(const std::vector<Eigen::Vector3f>& ground_points,
                                            Eigen::Vector3f& point,
                                            Eigen::Vector3f& direction) const
{
  if (ground_points.size() < 2)
  {
    return false;
  }

  // Centroid.
  point = Eigen::Vector3f::Zero();
  for (const auto& p : ground_points)
  {
    point += p;
  }
  point /= static_cast<float>(ground_points.size());

  // Covariance matrix.
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (const auto& p : ground_points)
  {
    const Eigen::Vector3f diff = p - point;
    cov += diff * diff.transpose();
  }

  // Principal component.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
  if (solver.info() != Eigen::Success)
  {
    return false;
  }

  // Eigenvalues are sorted in increasing order by SelfAdjointEigenSolver.
  direction = solver.eigenvectors().col(2);
  if (direction.norm() < 1e-6f)
  {
    return false;
  }
  direction.normalize();

  // Force direction to point generally forward.
  // For a tilted camera this is the projection of camera +Z; for a vertical
  // camera (nearly parallel to ground normal) we fall back to image "up"
  // (-camera Y), which corresponds to vehicle forward when the camera is
  // mounted with image up aligned to the vehicle forward direction.
  const Eigen::Vector3f cam_z = Eigen::Vector3f::UnitZ();
  Eigen::Vector3f forward = cam_z - cam_z.dot(ground_normal_) * ground_normal_;
  if (forward.norm() < 0.1f)
  {
    const Eigen::Vector3f cam_y_up = -Eigen::Vector3f::UnitY();
    forward = cam_y_up - cam_y_up.dot(ground_normal_) * ground_normal_;
  }
  if (forward.norm() > 1e-6f)
  {
    forward.normalize();
    if (direction.dot(forward) < 0.0f)
    {
      direction = -direction;
    }
  }
  else
  {
    ROS_WARN_THROTTLE(1.0,
                      "[DepthGuideLineEstimator] cannot determine ground "
                      "forward direction (camera Z and Y both parallel to "
                      "ground normal)");
  }

  return true;
}

double DepthGuideLineEstimator::computeYawError(const Eigen::Vector3f& ground_direction) const
{
  // Vehicle forward on the ground plane. Prefer camera +Z projection for
  // forward-looking cameras; fall back to image "up" (-camera Y) when the
  // camera is nearly vertical.
  const Eigen::Vector3f cam_z = Eigen::Vector3f::UnitZ();
  Eigen::Vector3f forward = cam_z - cam_z.dot(ground_normal_) * ground_normal_;
  if (forward.norm() < 0.1f)
  {
    const Eigen::Vector3f cam_y_up = -Eigen::Vector3f::UnitY();
    forward = cam_y_up - cam_y_up.dot(ground_normal_) * ground_normal_;
  }

  if (forward.norm() < 1e-6f)
  {
    ROS_WARN_THROTTLE(1.0,
                      "[DepthGuideLineEstimator] cannot determine vehicle "
                      "forward direction for yaw computation");
    return 0.0;
  }
  forward.normalize();

  // 'left' completes the right-handed basis (forward x left = up).
  const Eigen::Vector3f left = forward.cross(ground_normal_);

  const double angle = std::atan2(static_cast<double>(ground_direction.dot(left)),
                                  static_cast<double>(ground_direction.dot(forward)));

  // A guide line is geometrically undirected: both directions along the line
  // represent the same line. Pick the yaw error with the smaller magnitude.
  const double error1 = normalizeAngle(angle - target_angle_);
  const double error2 = normalizeAngle(angle + M_PI - target_angle_);
  return (std::abs(error1) < std::abs(error2)) ? error1 : error2;
}

double DepthGuideLineEstimator::computeLateralErrorM(const Eigen::Vector3f& ground_point,
                                                     const Eigen::Vector3f& ground_direction) const
{
  const Eigen::Vector3f camera_origin_on_ground = -ground_d_ * ground_normal_;

  // Vehicle forward on the ground plane, same logic as yaw computation.
  const Eigen::Vector3f cam_z = Eigen::Vector3f::UnitZ();
  Eigen::Vector3f forward = cam_z - cam_z.dot(ground_normal_) * ground_normal_;
  if (forward.norm() < 0.1f)
  {
    const Eigen::Vector3f cam_y_up = -Eigen::Vector3f::UnitY();
    forward = cam_y_up - cam_y_up.dot(ground_normal_) * ground_normal_;
  }

  if (forward.norm() < 1e-6f)
  {
    ROS_WARN_THROTTLE(1.0,
                      "[DepthGuideLineEstimator] cannot determine vehicle "
                      "forward direction for lateral error computation");
    return 0.0;
  }
  forward.normalize();

  const Eigen::Vector3f ref_point = camera_origin_on_ground + static_cast<float>(look_ahead_m_) * forward;

  // Signed distance from the reference point to the ground line.
  // 'perp' points to the left of the ground_direction.
  const Eigen::Vector3f perp = ground_direction.cross(ground_normal_);

  return static_cast<double>((ref_point - ground_point).dot(perp));
}

double DepthGuideLineEstimator::normalizeAngle(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace gemini_geometry_detector
