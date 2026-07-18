#ifndef GEMINI_GEOMETRY_DETECTOR_DEPTH_GUIDE_LINE_ESTIMATOR_H
#define GEMINI_GEOMETRY_DETECTOR_DEPTH_GUIDE_LINE_ESTIMATOR_H

#include <opencv2/core.hpp>
#include <Eigen/Dense>

#include "gemini_geometry_detector/guide_line_estimator_interface.h"

namespace gemini_geometry_detector
{

/**
 * @brief Depth/ground-plane guide-line estimator.
 *
 * Back-projects the 2D contour to rays, intersects them with the ground plane,
 * fits a line on the ground plane, and computes yaw/lateral errors in meters.
 */
class DepthGuideLineEstimator : public IGuideLineEstimator
{
public:
  DepthGuideLineEstimator() = default;
  ~DepthGuideLineEstimator() override = default;

  void configure(const CameraIntrinsics& intrinsics,
                 double image_scale,
                 double target_angle,
                 double roi_y_ratio,
                 int roi_y,
                 double look_ahead_m) override;

  void setGroundPlane(const Eigen::Vector3f& normal, float d) override;

  GuideLineError estimate(const std::vector<cv::Point>& largest_contour,
                          const std::vector<cv::Point>& rect_contour,
                          const cv::Size& image_size,
                          const std_msgs::Header& header,
                          cv::Mat& annotated) override;

private:
  bool pixelToRay(int u, int v, Eigen::Vector3f& ray) const;
  bool rayPlaneIntersection(const Eigen::Vector3f& ray,
                            const Eigen::Vector3f& normal,
                            float d,
                            Eigen::Vector3f& point) const;

  bool fitGroundLine(const std::vector<Eigen::Vector3f>& ground_points,
                     Eigen::Vector3f& point,
                     Eigen::Vector3f& direction) const;

  double computeYawError(const Eigen::Vector3f& ground_direction) const;
  double computeLateralErrorM(const Eigen::Vector3f& ground_point,
                              const Eigen::Vector3f& ground_direction) const;

  static double normalizeAngle(double angle);

  CameraIntrinsics camera_intrinsics_;
  double image_scale_ = 1.0;
  double target_angle_ = 0.0;
  double look_ahead_m_ = 0.0;

  Eigen::Vector3f ground_normal_ = Eigen::Vector3f::UnitZ();
  float ground_d_ = 0.0f;
  bool ground_plane_received_ = false;

  int max_contour_points_ = 2048;
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_DEPTH_GUIDE_LINE_ESTIMATOR_H
