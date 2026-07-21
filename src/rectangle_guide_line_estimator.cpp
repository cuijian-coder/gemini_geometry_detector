#include "gemini_geometry_detector/rectangle_guide_line_estimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <ros/ros.h>

namespace gemini_geometry_detector
{

void RectangleGuideLineEstimator::configure(const CameraIntrinsics& intrinsics,
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

  ROS_INFO("[RectangleGuideLineEstimator] configured:");
  ROS_INFO("  scaled intrinsics: fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
           camera_intrinsics_.fx, camera_intrinsics_.fy,
           camera_intrinsics_.cx, camera_intrinsics_.cy);
  ROS_INFO("  image_scale:       %.2f", image_scale_);
  ROS_INFO("  target_angle:      %.4f rad (%.1f deg)",
           target_angle_, target_angle_ * 180.0 / M_PI);
  ROS_INFO("  look_ahead_m:      %.3f m", look_ahead_m_);
  ROS_INFO("  guide_line_width_m: %.3f m", guide_line_width_m_);
  ROS_INFO("  width_tolerance_m:  %.3f m", width_tolerance_m_);
  ROS_INFO("  max_outlier_ratio:  %.2f", max_outlier_ratio_);
  ROS_INFO("  min_ground_points:  %d", min_ground_points_);
}

void RectangleGuideLineEstimator::setObbParameters(double guide_line_width_m,
                                                   double width_tolerance_m,
                                                   double max_outlier_ratio,
                                                   int min_ground_points)
{
  guide_line_width_m_ = guide_line_width_m;
  width_tolerance_m_ = width_tolerance_m;
  max_outlier_ratio_ = max_outlier_ratio;
  min_ground_points_ = min_ground_points;

  if (guide_line_width_m_ <= 0.0)
  {
    ROS_WARN("guide_line_width_m must be > 0, reset to 0.10");
    guide_line_width_m_ = 0.10;
  }
  if (width_tolerance_m_ < 0.0)
  {
    ROS_WARN("obb_width_tolerance_m must be >= 0, reset to 0.05");
    width_tolerance_m_ = 0.05;
  }
  if (max_outlier_ratio_ < 0.0 || max_outlier_ratio_ > 1.0)
  {
    ROS_WARN("obb_max_outlier_ratio must be in [0, 1], reset to 0.30");
    max_outlier_ratio_ = 0.30;
  }
  if (min_ground_points_ < 2)
  {
    ROS_WARN("obb_min_ground_points must be >= 2, reset to 10");
    min_ground_points_ = 10;
  }
}

void RectangleGuideLineEstimator::setGroundPlane(const Eigen::Vector3f& normal, float d)
{
  ground_normal_ = normal;
  if (ground_normal_.norm() > 1e-6f)
  {
    ground_normal_.normalize();
  }
  ground_d_ = d;

  // Same sign convention as DepthGuideLineEstimator.
  if (ground_d_ < 0.0f)
  {
    ground_normal_ = -ground_normal_;
    ground_d_ = -ground_d_;
  }

  ground_plane_received_ = true;

  static bool first = true;
  if (first)
  {
    ROS_INFO("[RectangleGuideLineEstimator] first ground plane set: "
             "n=(%.3f, %.3f, %.3f), d=%.3f",
             ground_normal_.x(), ground_normal_.y(),
             ground_normal_.z(), ground_d_);
    first = false;
  }

  ROS_INFO_THROTTLE(5.0,
                    "[RectangleGuideLineEstimator] ground plane: "
                    "n=(%.3f, %.3f, %.3f), d=%.3f",
                    ground_normal_.x(), ground_normal_.y(),
                    ground_normal_.z(), ground_d_);
}

GuideLineError RectangleGuideLineEstimator::estimate(const std::vector<cv::Point>& largest_contour,
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
    ROS_WARN_THROTTLE(1.0,
                      "[RectangleGuideLineEstimator] ground plane not received yet");
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

  if (ground_points.size() < static_cast<size_t>(min_ground_points_))
  {
    ROS_WARN_THROTTLE(1.0,
                      "[RectangleGuideLineEstimator] not enough ground-plane points: "
                      "%zu valid (contour_pts=%zu, skipped_rays=%d, "
                      "skipped_intersections=%d). "
                      "Ground plane: n=(%.3f, %.3f, %.3f), d=%.3f.",
                      ground_points.size(), largest_contour.size(),
                      skipped_rays, skipped_intersections,
                      ground_normal_.x(), ground_normal_.y(),
                      ground_normal_.z(), ground_d_);
    return error_msg;
  }

  Eigen::Vector3f ground_point;
  Eigen::Vector3f ground_direction;
  if (!fitGroundRectangle(ground_points, ground_point, ground_direction))
  {
    ROS_WARN_THROTTLE(1.0,
                      "[RectangleGuideLineEstimator] failed to fit ground rectangle "
                      "from %zu points", ground_points.size());
    return error_msg;
  }

  const double yaw_error = computeYawError(ground_direction);
  const double lateral_error_m = computeLateralErrorM(ground_point, ground_direction);

  error_msg.valid = true;
  error_msg.yaw_error = yaw_error;
  error_msg.lateral_error_n = 0.0;  // Not used by this estimator.
  error_msg.lateral_error_m = lateral_error_m;

  // Reference ROI point: camera origin on ground, projected forward by look_ahead_m.
  const Eigen::Vector3f cam_z = Eigen::Vector3f::UnitZ();
  Eigen::Vector3f forward = cam_z - cam_z.dot(ground_normal_) * ground_normal_;
  if (forward.norm() < 1e-6f)
  {
    const Eigen::Vector3f cam_y = Eigen::Vector3f::UnitY();
    forward = cam_y - cam_y.dot(ground_normal_) * ground_normal_;
  }
  forward.normalize();

  const Eigen::Vector3f ref_point =
      (-ground_d_ * ground_normal_) + static_cast<float>(look_ahead_m_) * forward;
  if (std::fabs(ref_point.z()) > 1e-6f)
  {
    error_msg.roi_point.x = ref_point.x() / ref_point.z();
    error_msg.roi_point.y = ref_point.y() / ref_point.z();
    error_msg.roi_point.z = ref_point.z();
  }

  // Draw the fitted center axis on the annotated image.
  cv::Point2f uv_center;
  if (projectToImage(ground_point, uv_center))
  {
    // Draw a short segment centered on the OBB center.
    const float half_len = 0.2f;  // 20 cm visual length.
    const Eigen::Vector3f p0 = ground_point - half_len * ground_direction;
    const Eigen::Vector3f p1 = ground_point + half_len * ground_direction;
    cv::Point2f uv0, uv1;
    if (projectToImage(p0, uv0) && projectToImage(p1, uv1))
    {
      cv::line(annotated, uv0, uv1, cv::Scalar(0, 255, 0), 2);
      cv::circle(annotated, uv_center, 4, cv::Scalar(0, 255, 0), -1);
    }
  }

  ROS_INFO_THROTTLE(1.0,
                    "[RectangleGuideLineEstimator] result: contour_pts=%zu, "
                    "ground_pts=%zu, skipped_rays=%d, skipped_intersections=%d, "
                    "yaw=%.4f rad (%.1f deg), lateral=%.4f m, look_ahead=%.3f m",
                    largest_contour.size(), ground_points.size(),
                    skipped_rays, skipped_intersections,
                    yaw_error, yaw_error * 180.0 / M_PI,
                    lateral_error_m, look_ahead_m_);

  return error_msg;
}

bool RectangleGuideLineEstimator::pixelToRay(int u, int v, Eigen::Vector3f& ray) const
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

bool RectangleGuideLineEstimator::rayPlaneIntersection(const Eigen::Vector3f& ray,
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

bool RectangleGuideLineEstimator::fitGroundRectangle(
    const std::vector<Eigen::Vector3f>& ground_points,
    Eigen::Vector3f& center,
    Eigen::Vector3f& direction) const
{
  if (ground_points.size() < static_cast<size_t>(min_ground_points_))
  {
    return false;
  }

  // 1. Centroid.
  center = Eigen::Vector3f::Zero();
  for (const auto& p : ground_points)
  {
    center += p;
  }
  center /= static_cast<float>(ground_points.size());

  // 2. Covariance.
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  for (const auto& p : ground_points)
  {
    const Eigen::Vector3f diff = p - center;
    cov += diff * diff.transpose();
  }

  // 3. Eigen decomposition.
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(cov);
  if (solver.info() != Eigen::Success)
  {
    return false;
  }

  // Eigenvalues are sorted in increasing order.
  // col(2) = long axis (largest variance), col(1) = short axis, col(0) ≈ normal.
  const Eigen::Vector3f long_axis = solver.eigenvectors().col(2);
  const Eigen::Vector3f short_axis = solver.eigenvectors().col(1);

  if (long_axis.norm() < 1e-6f || short_axis.norm() < 1e-6f)
  {
    return false;
  }

  // 4. Project points onto the two axes and measure extents.
  float u_min = std::numeric_limits<float>::max();
  float u_max = -std::numeric_limits<float>::max();
  float v_min = std::numeric_limits<float>::max();
  float v_max = -std::numeric_limits<float>::max();
  for (const auto& p : ground_points)
  {
    const Eigen::Vector3f diff = p - center;
    const float u = diff.dot(long_axis);
    const float v = diff.dot(short_axis);
    u_min = std::min(u_min, u);
    u_max = std::max(u_max, u);
    v_min = std::min(v_min, v);
    v_max = std::max(v_max, v);
  }

  const float actual_width = v_max - v_min;
  const float expected_width = static_cast<float>(guide_line_width_m_);

  // 5. Soft width check (warn but do not reject).
  if (std::fabs(actual_width - expected_width) > static_cast<float>(width_tolerance_m_))
  {
    ROS_WARN_THROTTLE(1.0,
                      "[RectangleGuideLineEstimator] OBB short-axis width "
                      "%.3f m differs from expected %.3f m (tolerance %.3f m). "
                      "Result kept but may be unreliable.",
                      actual_width, expected_width, width_tolerance_m_);
  }

  // 6. Outlier check: points outside the expected band around the long axis.
  const float half_expected_width = expected_width * 0.5f;
  const float outlier_threshold = half_expected_width + static_cast<float>(width_tolerance_m_);
  int outlier_count = 0;
  for (const auto& p : ground_points)
  {
    const float v = (p - center).dot(short_axis);
    if (std::fabs(v) > outlier_threshold)
    {
      ++outlier_count;
    }
  }
  const float outlier_ratio = static_cast<float>(outlier_count) / ground_points.size();
  if (outlier_ratio > static_cast<float>(max_outlier_ratio_))
  {
    ROS_WARN_THROTTLE(1.0,
                      "[RectangleGuideLineEstimator] outlier ratio %.2f exceeds "
                      "threshold %.2f. Result kept but quality may be poor.",
                      outlier_ratio, max_outlier_ratio_);
  }

  // 7. Align direction with vehicle forward.
  direction = long_axis.normalized();
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
                      "[RectangleGuideLineEstimator] cannot determine ground "
                      "forward direction (camera Z and Y both parallel to "
                      "ground normal)");
  }

  return true;
}

double RectangleGuideLineEstimator::computeYawError(const Eigen::Vector3f& ground_direction) const
{
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
                      "[RectangleGuideLineEstimator] cannot determine vehicle "
                      "forward direction for yaw computation");
    return 0.0;
  }
  forward.normalize();

  const Eigen::Vector3f left = forward.cross(ground_normal_);

  const double angle = std::atan2(static_cast<double>(ground_direction.dot(left)),
                                  static_cast<double>(ground_direction.dot(forward)));

  const double error1 = normalizeAngle(angle - target_angle_);
  const double error2 = normalizeAngle(angle + M_PI - target_angle_);
  return (std::abs(error1) < std::abs(error2)) ? error1 : error2;
}

double RectangleGuideLineEstimator::computeLateralErrorM(
    const Eigen::Vector3f& ground_point,
    const Eigen::Vector3f& ground_direction) const
{
  const Eigen::Vector3f camera_origin_on_ground = -ground_d_ * ground_normal_;

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
                      "[RectangleGuideLineEstimator] cannot determine vehicle "
                      "forward direction for lateral error computation");
    return 0.0;
  }
  forward.normalize();

  const Eigen::Vector3f ref_point =
      camera_origin_on_ground + static_cast<float>(look_ahead_m_) * forward;

  const Eigen::Vector3f perp = ground_direction.cross(ground_normal_);

  return static_cast<double>((ref_point - ground_point).dot(perp));
}

bool RectangleGuideLineEstimator::projectToImage(const Eigen::Vector3f& point,
                                                 cv::Point2f& uv) const
{
  if (point.z() <= 1e-6f)
  {
    return false;
  }

  uv.x = camera_intrinsics_.fx * (point.x() / point.z()) + camera_intrinsics_.cx;
  uv.y = camera_intrinsics_.fy * (point.y() / point.z()) + camera_intrinsics_.cy;

  // Accept even if slightly outside the image; caller can clip if needed.
  return true;
}

double RectangleGuideLineEstimator::normalizeAngle(double angle)
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
