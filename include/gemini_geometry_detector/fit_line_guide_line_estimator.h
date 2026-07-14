#ifndef GEMINI_GEOMETRY_DETECTOR_FIT_LINE_GUIDE_LINE_ESTIMATOR_H
#define GEMINI_GEOMETRY_DETECTOR_FIT_LINE_GUIDE_LINE_ESTIMATOR_H

#include <opencv2/core.hpp>

#include "gemini_geometry_detector/guide_line_estimator_interface.h"

namespace gemini_geometry_detector
{

/**
 * @brief Guide-line estimator based on cv::fitLine in the normalized image
 *        plane.
 *
 * Algorithm:
 *   1. Normalize contour points using CameraInfo.
 *   2. Fit a line with cv::fitLine.
 *   3. Compute yaw_error relative to the configured target angle.
 *   4. Evaluate the line at the configured ROI row to get lateral_error_n.
 *   5. Draw the fitted line, ROI, intersection point and error text.
 */
class FitLineGuideLineEstimator : public IGuideLineEstimator
{
public:
  FitLineGuideLineEstimator() = default;
  ~FitLineGuideLineEstimator() override = default;

  void configure(const CameraIntrinsics& intrinsics,
                 double image_scale,
                 double target_angle,
                 double roi_y_ratio,
                 int roi_y,
                 double look_ahead_m) override;

  GuideLineError estimate(const std::vector<cv::Point>& largest_contour,
                          const std::vector<cv::Point>& rect_contour,
                          const cv::Size& image_size,
                          const std_msgs::Header& header,
                          cv::Mat& annotated) override;

private:
  bool normalizeContourPoints(const std::vector<cv::Point>& contour,
                              std::vector<cv::Point2f>& normalized_points) const;

  bool fitCenterLineNormalized(const std::vector<cv::Point>& contour,
                               cv::Vec4f& line) const;

  double computeYawError(const cv::Vec4f& line) const;

  static double normalizeAngle(double angle);

  bool computeLateralErrorN(const cv::Vec4f& line,
                            int image_rows,
                            double& lateral_error_n,
                            double& roi_x_pixel,
                            int& roi_y_pixel) const;

  void drawGuideLineErrorOnImage(cv::Mat& annotated,
                                 const std::vector<cv::Point>& rect_contour,
                                 const cv::Vec4f& line,
                                 int roi_y_pixel,
                                 double roi_x_pixel,
                                 double yaw_error,
                                 double lateral_error_n) const;

  CameraIntrinsics camera_intrinsics_;
  double image_scale_ = 1.0;
  double target_angle_ = -M_PI / 2.0;
  double roi_y_ratio_ = 0.75;
  int roi_y_ = -1;
  int max_contour_points_ = 64;
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_FIT_LINE_GUIDE_LINE_ESTIMATOR_H
