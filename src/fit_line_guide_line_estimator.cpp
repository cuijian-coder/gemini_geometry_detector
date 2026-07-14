#include "gemini_geometry_detector/fit_line_guide_line_estimator.h"

#include <ros/ros.h>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <limits>
#include <sstream>

namespace gemini_geometry_detector
{

void FitLineGuideLineEstimator::configure(const CameraIntrinsics& intrinsics,
                                          double image_scale,
                                          double target_angle,
                                          double roi_y_ratio,
                                          int roi_y,
                                          double /*look_ahead_m*/)
{
  camera_intrinsics_ = intrinsics;
  image_scale_ = image_scale;
  target_angle_ = target_angle;
  roi_y_ratio_ = roi_y_ratio;
  roi_y_ = roi_y;

  ROS_INFO("[FitLineGuideLineEstimator] configured:");
  ROS_INFO("  image_scale:   %.2f", image_scale_);
  ROS_INFO("  target_angle:  %.4f rad (%.1f deg)",
           target_angle_, target_angle_ * 180.0 / M_PI);
  ROS_INFO("  roi_y_ratio:   %.2f", roi_y_ratio_);
  ROS_INFO("  roi_y:         %d (%s)", roi_y_,
           roi_y_ < 0 ? "use ratio" : "absolute pixels");
}

GuideLineError FitLineGuideLineEstimator::estimate(const std::vector<cv::Point>& largest_contour,
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

  // Draw the rotated rectangle for visualization only.
  if (!rect_contour.empty())
  {
    cv::polylines(annotated, std::vector<std::vector<cv::Point>>{rect_contour},
                  true, cv::Scalar(0, 255, 255), 2);
  }

  cv::Vec4f line;
  if (!fitCenterLineNormalized(largest_contour, line))
  {
    ROS_WARN_THROTTLE(1.0,
                      "[FitLineGuideLineEstimator] failed to fit center line "
                      "(contour points: %zu)",
                      largest_contour.size());
    return error_msg;
  }

  double lateral_error_n = 0.0;
  double roi_x_pixel = 0.0;
  int roi_y_pixel = 0;
  if (!computeLateralErrorN(line, image_size.height,
                            lateral_error_n, roi_x_pixel, roi_y_pixel))
  {
    ROS_WARN_THROTTLE(1.0,
                      "[FitLineGuideLineEstimator] failed to compute lateral error "
                      "(image height: %d)",
                      image_size.height);
    return error_msg;
  }

  const double yaw_error = computeYawError(line);

  error_msg.valid = true;
  error_msg.yaw_error = yaw_error;
  error_msg.lateral_error_n = lateral_error_n;
  error_msg.lateral_error_m = 0.0;  // Not used by RGB-only estimator.
  // ROI point in original image coordinates.
  error_msg.roi_point.x = roi_x_pixel / image_scale_;
  error_msg.roi_point.y = roi_y_pixel / image_scale_;
  error_msg.roi_point.z = 0.0;

  drawGuideLineErrorOnImage(annotated, rect_contour, line,
                            roi_y_pixel, roi_x_pixel,
                            yaw_error, lateral_error_n);

  if (roi_x_pixel < 0.0 || roi_x_pixel >= static_cast<double>(image_size.width))
  {
    ROS_WARN_THROTTLE(1.0,
                      "[FitLineGuideLineEstimator] ROI intersection x=%.1f is outside "
                      "image width [0, %d]. Consider adjusting roi_y_ratio / roi_y.",
                      roi_x_pixel, image_size.width);
  }

  ROS_INFO_THROTTLE(1.0,
                    "[FitLineGuideLineEstimator] result: contour_pts=%zu, "
                    "line=(vx=%.3f, vy=%.3f, x0=%.1f, y0=%.1f), "
                    "yaw=%.4f rad (%.1f deg), lat_n=%.4f, roi=(%.1f, %.1f)",
                    largest_contour.size(),
                    static_cast<double>(line[0]),
                    static_cast<double>(line[1]),
                    static_cast<double>(line[2]),
                    static_cast<double>(line[3]),
                    yaw_error, yaw_error * 180.0 / M_PI,
                    lateral_error_n,
                    error_msg.roi_point.x, error_msg.roi_point.y);

  return error_msg;
}

bool FitLineGuideLineEstimator::normalizeContourPoints(
    const std::vector<cv::Point>& contour,
    std::vector<cv::Point2f>& normalized_points) const
{
  normalized_points.clear();
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

  normalized_points.reserve(contour.size() / step + 1);
  for (size_t i = 0; i < contour.size(); i += step)
  {
    const float x_n = (static_cast<float>(contour[i].x) - camera_intrinsics_.cx) /
                      camera_intrinsics_.fx;
    const float y_n = (static_cast<float>(contour[i].y) - camera_intrinsics_.cy) /
                      camera_intrinsics_.fy;
    normalized_points.emplace_back(x_n, y_n);
  }

  return !normalized_points.empty();
}

bool FitLineGuideLineEstimator::fitCenterLineNormalized(const std::vector<cv::Point>& contour,
                                                        cv::Vec4f& line) const
{
  if (!camera_intrinsics_.valid())
  {
    ROS_WARN_THROTTLE(1.0,
                      "Cannot estimate guide line: camera_info not received yet or invalid");
    return false;
  }

  std::vector<cv::Point2f> normalized_points;
  if (!normalizeContourPoints(contour, normalized_points))
  {
    return false;
  }

  cv::fitLine(normalized_points, line, cv::DIST_L2, 0, 0.01, 0.01);
  return true;
}

double FitLineGuideLineEstimator::normalizeAngle(double angle)
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

double FitLineGuideLineEstimator::computeYawError(const cv::Vec4f& line) const
{
  const double angle = std::atan2(static_cast<double>(line[1]),
                                  static_cast<double>(line[0]));

  // A line has no inherent direction; cv::fitLine can return either of the
  // two opposite vectors along the same line. Pick the signed error with the
  // smallest magnitude so the result is stable and independent of the fitted
  // vector orientation.
  const double error1 = normalizeAngle(angle - target_angle_);
  const double error2 = normalizeAngle(angle + M_PI - target_angle_);

  return (std::abs(error1) < std::abs(error2)) ? error1 : error2;
}

bool FitLineGuideLineEstimator::computeLateralErrorN(const cv::Vec4f& line,
                                                     int image_rows,
                                                     double& lateral_error_n,
                                                     double& roi_x_pixel,
                                                     int& roi_y_pixel) const
{
  if (!camera_intrinsics_.valid())
  {
    return false;
  }

  int roi_y_input = roi_y_;
  if (roi_y_input < 0)
  {
    roi_y_input = static_cast<int>(roi_y_ratio_ * static_cast<double>(image_rows));
  }
  roi_y_input = std::max(0, std::min(roi_y_input, image_rows - 1));
  roi_y_pixel = roi_y_input;

  const double roi_y_n =
      (static_cast<double>(roi_y_pixel) - static_cast<double>(camera_intrinsics_.cy)) /
      static_cast<double>(camera_intrinsics_.fy);

  const double vx = static_cast<double>(line[0]);
  const double vy = static_cast<double>(line[1]);
  if (std::abs(vy) < 1e-6)
  {
    ROS_WARN_THROTTLE(1.0, "Guide line is horizontal, cannot evaluate at ROI");
    return false;
  }

  const double x0_n = static_cast<double>(line[2]);
  const double y0_n = static_cast<double>(line[3]);

  const double x_roi_n = x0_n + (roi_y_n - y0_n) * vx / vy;
  lateral_error_n = x_roi_n;

  // Convert back to pixel coords for visualization.
  roi_x_pixel = x_roi_n * static_cast<double>(camera_intrinsics_.fx) +
                static_cast<double>(camera_intrinsics_.cx);

  return std::isfinite(lateral_error_n);
}

void FitLineGuideLineEstimator::drawGuideLineErrorOnImage(
    cv::Mat& annotated,
    const std::vector<cv::Point>& rect_contour,
    const cv::Vec4f& line,
    int roi_y_pixel,
    double roi_x_pixel,
    double yaw_error,
    double lateral_error_n) const
{
  (void)rect_contour;  // Already drawn in estimate().

  const int rows = annotated.rows;
  const int cols = annotated.cols;

  const double vx = static_cast<double>(line[0]);
  const double vy = static_cast<double>(line[1]);
  const double x0_n = static_cast<double>(line[2]);
  const double y0_n = static_cast<double>(line[3]);

  if (std::abs(vy) > 1e-6 && camera_intrinsics_.valid())
  {
    const double dfx = static_cast<double>(camera_intrinsics_.fx);
    const double dfy = static_cast<double>(camera_intrinsics_.fy);
    const double dcx = static_cast<double>(camera_intrinsics_.cx);
    const double dcy = static_cast<double>(camera_intrinsics_.cy);

    // Evaluate fitted line at top and bottom of image for drawing.
    const double y_top_n = (0.0 - dcy) / dfy;
    const double y_bottom_n = (static_cast<double>(rows - 1) - dcy) / dfy;

    const double x_top_n = x0_n + (y_top_n - y0_n) * vx / vy;
    const double x_bottom_n = x0_n + (y_bottom_n - y0_n) * vx / vy;

    const int x_top = static_cast<int>(x_top_n * dfx + dcx);
    const int x_bottom = static_cast<int>(x_bottom_n * dfx + dcx);

    cv::line(annotated, cv::Point(x_top, 0), cv::Point(x_bottom, rows - 1),
             cv::Scalar(0, 255, 0), 2);
  }

  // ROI horizontal line.
  cv::line(annotated, cv::Point(0, roi_y_pixel), cv::Point(cols - 1, roi_y_pixel),
           cv::Scalar(0, 255, 255), 1);

  // Intersection point.
  cv::circle(annotated, cv::Point(static_cast<int>(roi_x_pixel), roi_y_pixel),
             5, cv::Scalar(0, 0, 255), -1);

  // Image center marker on ROI row.
  const int center_x = cols / 2;
  cv::circle(annotated, cv::Point(center_x, roi_y_pixel),
             3, cv::Scalar(255, 0, 0), -1);

  // Lateral error line segment between image center and ROI intersection.
  // Clip to image bounds so the visualization stays on screen even if the
  // fitted line extrapolates outside.
  const int clipped_roi_x =
      std::max(0, std::min(cols - 1, static_cast<int>(roi_x_pixel)));
  cv::line(annotated,
           cv::Point(center_x, roi_y_pixel),
           cv::Point(clipped_roi_x, roi_y_pixel),
           cv::Scalar(255, 0, 255), 2);

  // Text.
  std::ostringstream yaw_ss;
  yaw_ss << std::fixed << std::setprecision(3) << "yaw: " << yaw_error << " rad";
  cv::putText(annotated, yaw_ss.str(), cv::Point(10, 30),
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

  std::ostringstream lat_ss;
  lat_ss << std::fixed << std::setprecision(4) << "lat: " << lateral_error_n;
  cv::putText(annotated, lat_ss.str(), cv::Point(10, 60),
              cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);
}

}  // namespace gemini_geometry_detector
