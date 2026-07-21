#ifndef GEMINI_GEOMETRY_DETECTOR_RECTANGLE_GUIDE_LINE_ESTIMATOR_H
#define GEMINI_GEOMETRY_DETECTOR_RECTANGLE_GUIDE_LINE_ESTIMATOR_H

#include <opencv2/core.hpp>
#include <Eigen/Dense>

#include "gemini_geometry_detector/guide_line_estimator_interface.h"

namespace gemini_geometry_detector
{

/**
 * @brief Ground-plane guide-line estimator based on oriented bounding box (OBB).
 *
 * Back-projects the 2D contour to rays, intersects them with the ground plane,
 * fits an oriented bounding box to the ground-plane point cloud, and uses the
 * long axis of the box as the guide-line direction.
 *
 * This is more robust than a pure PCA line fit when the contour is jittery or
 * irregular, because the OBB explicitly models the rectangular shape of a
 * physical guide line.
 */
class RectangleGuideLineEstimator : public IGuideLineEstimator
{
public:
  RectangleGuideLineEstimator() = default;
  ~RectangleGuideLineEstimator() override = default;

  void configure(const CameraIntrinsics& intrinsics,
                 double image_scale,
                 double target_angle,
                 double /*roi_y_ratio*/,
                 int /*roi_y*/,
                 double look_ahead_m) override;

  void setGroundPlane(const Eigen::Vector3f& normal, float d) override;

  /**
   * @brief Set OBB-related parameters.  Must be called before configure().
   */
  void setObbParameters(double guide_line_width_m,
                        double width_tolerance_m,
                        double max_outlier_ratio,
                        int min_ground_points);

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

  /**
   * @brief Fit an oriented bounding box to ground points and return its center
   *        and long-axis direction.
   *
   * The short-axis width is compared against the expected guide-line width and
   * warnings are emitted if it deviates, but the result is still returned.
   *
   * @param[in]  ground_points Ground-plane points from the contour.
   * @param[out] center        OBB center (centroid).
   * @param[out] direction     Unit vector along the OBB long axis, aligned
   *                          generally forward.
   * @return True if a valid box could be fitted.
   */
  bool fitGroundRectangle(const std::vector<Eigen::Vector3f>& ground_points,
                          Eigen::Vector3f& center,
                          Eigen::Vector3f& direction) const;

  double computeYawError(const Eigen::Vector3f& ground_direction) const;
  double computeLateralErrorM(const Eigen::Vector3f& ground_point,
                              const Eigen::Vector3f& ground_direction) const;

  /**
   * @brief Project a 3D ground point to scaled image coordinates.
   * @return True if the point is in front of the camera and projection succeeds.
   */
  bool projectToImage(const Eigen::Vector3f& point, cv::Point2f& uv) const;

  static double normalizeAngle(double angle);

  CameraIntrinsics camera_intrinsics_;
  double image_scale_ = 1.0;
  double target_angle_ = 0.0;
  double look_ahead_m_ = 0.0;

  // Ground plane: normal . P + d = 0.
  Eigen::Vector3f ground_normal_ = Eigen::Vector3f::UnitZ();
  float ground_d_ = 0.0f;
  bool ground_plane_received_ = false;

  // OBB parameters.
  double guide_line_width_m_ = 0.10;
  double width_tolerance_m_ = 0.05;
  double max_outlier_ratio_ = 0.30;
  int min_ground_points_ = 10;
  int max_contour_points_ = 2048;
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_RECTANGLE_GUIDE_LINE_ESTIMATOR_H
