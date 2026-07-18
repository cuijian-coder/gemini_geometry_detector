#ifndef GEMINI_GEOMETRY_DETECTOR_COLOR_REGION_DETECTOR_H
#define GEMINI_GEOMETRY_DETECTOR_COLOR_REGION_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>

#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "gemini_geometry_detector/camera_intrinsics.h"
#include "gemini_geometry_detector/guide_line_estimator_interface.h"
#include "gemini_geometry_detector/ground_plane_provider_interface.h"
#include "gemini_geometry_detector/ContourInfo.h"
#include "gemini_geometry_detector/ContourArray.h"
#include "gemini_geometry_detector/GuideLineError.h"

#include <opencv2/core/types.hpp>

namespace gemini_geometry_detector
{

/**
 * @brief Features of a contour used for merge decisions.
 *
 * All geometric quantities are in the normalized image plane, making the merge
 * thresholds independent of image resolution and focal length.
 */
struct NormalizedContourFeatures
{
  cv::Point2f center_n;  ///< Centroid in normalized image plane.
  double angle = 0.0;    ///< Line angle from cv::fitLine (rad).
  cv::Rect bbox;         ///< Axis-aligned bounding box in scaled image pixels.
  bool valid = false;    ///< True if the contour passed area/aspect filters.
};

/**
 * @brief Detects a colored guide region in the RGB image and estimates
 *        guide-line errors for visual servoing.
 *
 * The actual guide-line estimation algorithm is pluggable via
 * IGuideLineEstimator. Two implementations are provided:
 *   - "DepthGuideLineEstimator" (default): back-projects the contour using
 *     camera intrinsics and an external ground plane, and computes errors in
 *     meters on the ground plane.
 *   - "FitLineGuideLineEstimator": pure RGB image-plane fitting; computes
 *     normalized image-plane errors.
 *
 * The ground plane source for the depth estimator is pluggable via
 * IGroundPlaneProvider. Two implementations are provided:
 *   - "TopicGroundPlaneProvider" (default): listens to a
 *     ground_plane_calibrator/PlaneCoefficients topic.
 *   - "ImuGroundPlaneProvider": estimates the plane from IMU gravity and a
 *     fixed camera height.
 */
class ColorRegionDetector
{
public:
  explicit ColorRegionDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~ColorRegionDetector() = default;

private:
  void loadParameters();

  void rgbCallback(const sensor_msgs::ImageConstPtr& rgb_msg);

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg);

  void processFrame(const sensor_msgs::ImageConstPtr& rgb_msg,
                    const cv::Mat& bgr_image);

  /**
   * @brief Compute a minimum-area rotated rectangle from a contour.
   *
   * The resulting polygon is used only for visualization; control computations
   * are performed by the configured IGuideLineEstimator.
   */
  std::vector<cv::Point> getRectangularContour(const std::vector<cv::Point>& contour) const;

  /**
   * @brief Compute target_angle_ from TF once.
   *
   * Projects base_frame's +X axis (vehicle forward) into the camera image plane.
   * @return true if TF lookup and projection succeeded.
   */
  bool computeTargetAngleFromTf();

  /**
   * @brief Compute target_angle_ from TF if it has not been computed yet.
   */
  void ensureTargetAngleComputed();

  cv::Mat createColorMask(const cv::Mat& bgr_image);
  void applyMorphology(cv::Mat& mask);
  void detectContours(const cv::Mat& mask, std::vector<std::vector<cv::Point>>& contours);
  bool buildContourInfo(const std::vector<cv::Point>& contour,
                        int id,
                        ContourInfo& info,
                        double min_contour_length);

  /**
   * @brief Check whether the bounding box aspect ratio is within the configured
   *        range. Logs a warning if it is rejected.
   */
  bool isAspectRatioValid(const cv::Rect& bbox, int id) const;

  /**
   * @brief Compute normalized center and line direction for each contour.
   *
   * Only contours that pass area and aspect-ratio filtering are marked valid.
   */
  std::vector<NormalizedContourFeatures> computeContourFeatures(
      const std::vector<std::vector<cv::Point>>& contours,
      double min_contour_length) const;

  /**
   * @brief Group valid contours into merge groups and sort them from bottom to
   *        top of the image.
   *
   * When contour merging is enabled, contours are grouped with union-find based
   * on direction and normalized bounding-box gap.  Each group is a candidate
   * guide line, ordered so that the nearest (bottom-most) group is tried first.
   */
  std::vector<std::vector<size_t>> findBottomUpMergeGroups(
      const std::vector<NormalizedContourFeatures>& features,
      const std::vector<size_t>& valid_indices) const;

  /**
   * @brief Compute the nearest distance between two axis-aligned bounding boxes
   *        in the normalized image plane.
   *
   * Overlapping boxes return 0.0.
   */
  double computeNormalizedBboxGap(const cv::Rect& a, const cv::Rect& b) const;

  void drawContourOnImage(cv::Mat& annotated,
                          const std::vector<cv::Point>& contour,
                          const ContourInfo& info,
                          bool is_merged = false);

  void publishResults(const std_msgs::Header& header,
                      const cv::Mat& mask,
                      const cv::Mat& annotated,
                      const ContourArray& contour_array);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;

  image_transport::Subscriber rgb_sub_;
  image_transport::Publisher mask_pub_;
  image_transport::Publisher annotated_pub_;
  ros::Publisher contours_pub_;
  ros::Publisher guide_line_error_pub_;

  ros::Subscriber camera_info_sub_;

  // TF must be declared before TransformListener.
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unique_ptr<IGuideLineEstimator> guide_line_estimator_;
  std::unique_ptr<IGroundPlaneProvider> ground_plane_provider_;

  std::string input_topic_;
  std::string camera_info_topic_;
  std::string base_frame_;
  std::string camera_frame_;

  int h_min_, h_max_;
  int s_min_, s_max_;
  int v_min_, v_max_;
  int morph_kernel_size_;
  int min_contour_length_;
  int max_contour_points_;
  double min_aspect_ratio_;
  double max_aspect_ratio_;

  bool enable_contour_merging_;
  double merge_max_angle_diff_deg_;
  double merge_max_region_gap_n_;

  double image_scale_;
  double target_angle_;
  double roi_y_ratio_;
  int roi_y_;  // Absolute pixel row; < 0 means use roi_y_ratio_.
  double look_ahead_m_;

  std::string guide_line_estimator_type_;
  std::string ground_plane_provider_type_;

  int frame_counter_;
  bool first_rgb_received_;
  bool use_tf_target_angle_;
  bool target_angle_computed_;
  bool camera_info_received_;

  CameraIntrinsics camera_intrinsics_;
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_COLOR_REGION_DETECTOR_H
