#ifndef GEMINI_GEOMETRY_DETECTOR_GUIDE_LINE_ESTIMATOR_INTERFACE_H
#define GEMINI_GEOMETRY_DETECTOR_GUIDE_LINE_ESTIMATOR_INTERFACE_H

#include <opencv2/core.hpp>
#include <std_msgs/Header.h>
#include <Eigen/Dense>

#include "gemini_geometry_detector/camera_intrinsics.h"
#include "gemini_geometry_detector/GuideLineError.h"

namespace gemini_geometry_detector
{

/**
 * @brief Interface for algorithms that estimate a guide-line error from a 2D
 *        color contour.
 *
 * Implementations are configured once with camera intrinsics and algorithm
 * parameters, then called per frame with the detected contour.
 */
class IGuideLineEstimator
{
public:
  virtual ~IGuideLineEstimator() = default;

  /**
   * @brief Configure the estimator with intrinsics and runtime parameters.
   *
   * @param look_ahead_m  Distance ahead of the camera origin on the ground
   *                      plane at which to evaluate lateral error. Used by
   *                      depth-based estimators; RGB-only estimators ignore it.
   */
  virtual void configure(const CameraIntrinsics& intrinsics,
                         double image_scale,
                         double target_angle,
                         double roi_y_ratio,
                         int roi_y,
                         double look_ahead_m) = 0;

  /**
   * @brief Optional: provide the ground plane for depth-based estimators.
   *
   * RGB-only estimators can ignore this method.
   */
  virtual void setGroundPlane(const Eigen::Vector3f& /*normal*/, float /*d*/) {}

  /**
   * @brief Estimate the guide-line error from the largest contour.
   *
   * The implementation may draw visualization primitives on @p annotated.
   *
   * @param[in]  largest_contour  The largest valid contour in pixel coords.
   * @param[in]  rect_contour     Minimum-area rectangle of the contour, used
   *                              for visualization only.
   * @param[in]  image_size       Size of the processed (scaled) image.
   * @param[in]  header           Header to attach to the output message.
   * @param[in,out] annotated     Annotated image; may be drawn on.
   * @return A fully populated GuideLineError message.
   */
  virtual GuideLineError estimate(const std::vector<cv::Point>& largest_contour,
                                  const std::vector<cv::Point>& rect_contour,
                                  const cv::Size& image_size,
                                  const std_msgs::Header& header,
                                  cv::Mat& annotated) = 0;
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_GUIDE_LINE_ESTIMATOR_INTERFACE_H
