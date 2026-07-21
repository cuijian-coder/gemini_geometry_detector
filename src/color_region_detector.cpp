#include "gemini_geometry_detector/color_region_detector.h"

#include "gemini_geometry_detector/fit_line_guide_line_estimator.h"
#include "gemini_geometry_detector/depth_guide_line_estimator.h"
#include "gemini_geometry_detector/rectangle_guide_line_estimator.h"
#include "gemini_geometry_detector/topic_ground_plane_provider.h"
#include "gemini_geometry_detector/imu_ground_plane_provider.h"

#include <opencv2/imgproc.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <limits>
#include <numeric>
#include <map>

namespace
{

/**
 * @brief Minimal disjoint-set (union-find) for contour merge grouping.
 */
class UnionFind
{
public:
  explicit UnionFind(size_t n) : parent_(n)
  {
    std::iota(parent_.begin(), parent_.end(), 0);
  }

  size_t find(size_t x)
  {
    if (parent_[x] != x)
    {
      parent_[x] = find(parent_[x]);
    }
    return parent_[x];
  }

  void unite(size_t a, size_t b)
  {
    parent_[find(a)] = find(b);
  }

private:
  std::vector<size_t> parent_;
};

/**
 * @brief Normalize an angle difference to [0, pi/2] for line-direction comparison.
 */
double lineAngleDifference(double a, double b)
{
  double diff = std::abs(a - b);
  if (diff > M_PI)
  {
    diff = 2.0 * M_PI - diff;
  }
  // Line direction is ambiguous by pi.
  if (diff > M_PI / 2.0)
  {
    diff = M_PI - diff;
  }
  return diff;
}

}  // namespace

namespace gemini_geometry_detector
{

ColorRegionDetector::ColorRegionDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh), it_(nh),
    tf_listener_(tf_buffer_),
    frame_counter_(0),
    first_rgb_received_(false),
    use_tf_target_angle_(true),
    target_angle_computed_(false),
    camera_info_received_(false)
{
  loadParameters();

  if (guide_line_estimator_type_ == "FitLineGuideLineEstimator")
  {
    guide_line_estimator_.reset(new FitLineGuideLineEstimator());
    ROS_INFO("Using FitLineGuideLineEstimator (RGB-only image-plane fitting).");
  }
  else if (guide_line_estimator_type_ == "RectangleGuideLineEstimator")
  {
    auto* rect_estimator = new RectangleGuideLineEstimator();
    rect_estimator->setObbParameters(guide_line_width_m_,
                                     obb_width_tolerance_m_,
                                     obb_max_outlier_ratio_,
                                     obb_min_ground_points_);
    guide_line_estimator_.reset(rect_estimator);
    ROS_INFO("Using RectangleGuideLineEstimator (ground-plane OBB fitting).");
  }
  else
  {
    guide_line_estimator_.reset(new DepthGuideLineEstimator());
    ROS_INFO("Using DepthGuideLineEstimator (ground-plane 3D fitting).");
  }

  if (guide_line_estimator_type_ == "DepthGuideLineEstimator" ||
      guide_line_estimator_type_ == "RectangleGuideLineEstimator")
  {
    if (ground_plane_provider_type_ == "imu")
    {
      ground_plane_provider_.reset(new ImuGroundPlaneProvider());
      ROS_INFO("Using ImuGroundPlaneProvider (IMU gravity + fixed camera height).");
    }
    else
    {
      ground_plane_provider_.reset(new TopicGroundPlaneProvider());
      ROS_INFO("Using TopicGroundPlaneProvider (subscribed ground-plane topic).");
    }
    ground_plane_provider_->configure(pnh_);
  }

  mask_pub_ = it_.advertise("color/mask", 1);
  annotated_pub_ = it_.advertise("color/annotated", 1);
  contours_pub_ = nh_.advertise<ContourArray>("color/contours", 1);
  guide_line_error_pub_ = nh_.advertise<GuideLineError>("color/guide_line_error", 1);

  rgb_sub_ = it_.subscribe(input_topic_, 1,
                           &ColorRegionDetector::rgbCallback, this);
  camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1,
                                   &ColorRegionDetector::cameraInfoCallback, this);

  if (ground_plane_provider_)
  {
    ground_plane_provider_->start(nh_,
                                  [this](const Eigen::Vector3f& normal, float d)
                                  {
                                    guide_line_estimator_->setGroundPlane(normal, d);
                                  });
  }

  ROS_INFO("============================================================");
  ROS_INFO("ColorRegionDetector started");
  ROS_INFO("------------------------------------------------------------");
  ROS_INFO("[Input/output]");
  ROS_INFO("  RGB topic:        %s", input_topic_.c_str());
  ROS_INFO("  CameraInfo topic: %s", camera_info_topic_.c_str());
  ROS_INFO("  Output namespace: gemini_geometry_detector/color/*");
  ROS_INFO("[TF configuration]");
  ROS_INFO("  base_frame:          %s", base_frame_.c_str());
  ROS_INFO("  camera_frame:        %s", camera_frame_.c_str());
  ROS_INFO("  use_tf_target_angle: %s", use_tf_target_angle_ ? "true" : "false");
  ROS_INFO("[Guide-line estimator]");
  ROS_INFO("  type:         %s", guide_line_estimator_type_.c_str());
  ROS_INFO("  target_angle: %.4f rad (%.1f deg)",
           target_angle_, target_angle_ * 180.0 / M_PI);
  if (guide_line_estimator_type_ == "DepthGuideLineEstimator" ||
      guide_line_estimator_type_ == "RectangleGuideLineEstimator")
  {
    ROS_INFO("  look_ahead_m: %.3f m", look_ahead_m_);
    if (guide_line_estimator_type_ == "RectangleGuideLineEstimator")
    {
      ROS_INFO("  guide_line_width_m: %.3f m", guide_line_width_m_);
      ROS_INFO("  obb_width_tolerance_m: %.3f m", obb_width_tolerance_m_);
      ROS_INFO("  obb_max_outlier_ratio: %.2f", obb_max_outlier_ratio_);
      ROS_INFO("  obb_min_ground_points: %d", obb_min_ground_points_);
    }
    ROS_INFO("[Ground-plane provider]");
    ROS_INFO("  type: %s", ground_plane_provider_type_.c_str());
  }
  ROS_INFO("[Image preprocessing]");
  ROS_INFO("  image_scale:        %.2f", image_scale_);
  ROS_INFO("  roi_y_ratio:        %.2f", roi_y_ratio_);
  ROS_INFO("  roi_y:              %d (%s)", roi_y_,
           roi_y_ < 0 ? "use roi_y_ratio" : "absolute pixels");
  ROS_INFO("  roi_bottom_ratio:   %.2f", roi_bottom_ratio_);
  ROS_INFO("[Color segmentation (HSV)]");
  ROS_INFO("  H: [%3d, %3d]", h_min_, h_max_);
  ROS_INFO("  S: [%3d, %3d]", s_min_, s_max_);
  ROS_INFO("  V: [%3d, %3d]", v_min_, v_max_);
  ROS_INFO("[Contour filtering]");
  ROS_INFO("  morph_kernel_size:  %d", morph_kernel_size_);
  ROS_INFO("  min_contour_length: %d", min_contour_length_);
  ROS_INFO("  max_contour_points: %d", max_contour_points_);
  ROS_INFO("  aspect_ratio range: [%.2f, %s]",
           min_aspect_ratio_,
           max_aspect_ratio_ > 0.0 ? std::to_string(max_aspect_ratio_).c_str() : "inf");
  ROS_INFO("[Contour merging]");
  ROS_INFO("  enable:               %s", enable_contour_merging_ ? "true" : "false");
  ROS_INFO("  max_angle_diff_deg:   %.2f", merge_max_angle_diff_deg_);
  ROS_INFO("  max_region_gap_n:     %.3f", merge_max_region_gap_n_);
  ROS_INFO("[Mask temporal filtering]");
  ROS_INFO("  mask_filter_alpha:    %.3f (%s)", mask_filter_alpha_,
           mask_filter_alpha_ > 0.0 ? "enabled" : "disabled");
  ROS_INFO("[Error statistics overlay]");
  ROS_INFO("  enable:               %s", enable_error_stats_ ? "true" : "false");
  ROS_INFO("  window:               %d", error_stats_window_);
  ROS_INFO("============================================================");
}

void ColorRegionDetector::loadParameters()
{
  pnh_.param<std::string>("input_topic", input_topic_, "/camera/color/image_raw");
  pnh_.param<std::string>("camera_info_topic", camera_info_topic_, "/camera/color/camera_info");
  pnh_.param<std::string>("base_frame", base_frame_, "base_link");
  pnh_.param<std::string>("camera_frame", camera_frame_, "camera_color_optical_frame");

  pnh_.param("h_min", h_min_, 20);
  pnh_.param("h_max", h_max_, 40);
  pnh_.param("s_min", s_min_, 80);
  pnh_.param("s_max", s_max_, 255);
  pnh_.param("v_min", v_min_, 80);
  pnh_.param("v_max", v_max_, 255);

  pnh_.param("morph_kernel_size", morph_kernel_size_, 5);
  pnh_.param("min_contour_length", min_contour_length_, 50);
  pnh_.param("max_contour_points", max_contour_points_, 64);
  pnh_.param("min_aspect_ratio", min_aspect_ratio_, 1.0);
  pnh_.param("max_aspect_ratio", max_aspect_ratio_, 0.0);

  pnh_.param("enable_contour_merging", enable_contour_merging_, false);
  pnh_.param("merge_max_angle_diff_deg", merge_max_angle_diff_deg_, 15.0);
  pnh_.param("merge_max_region_gap_n", merge_max_region_gap_n_, 0.1);

  pnh_.param("mask_filter_alpha", mask_filter_alpha_, 0.0);

  pnh_.param("enable_error_stats", enable_error_stats_, true);
  pnh_.param("error_stats_window", error_stats_window_, 100);

  pnh_.param("image_scale", image_scale_, 0.5);
  pnh_.param("use_tf_target_angle", use_tf_target_angle_, true);
  pnh_.param("target_angle", target_angle_, -M_PI / 2.0);
  pnh_.param("roi_y_ratio", roi_y_ratio_, 0.75);
  pnh_.param("roi_y", roi_y_, -1);
  pnh_.param("roi_bottom_ratio", roi_bottom_ratio_, 0.5);
  pnh_.param("look_ahead_m", look_ahead_m_, 0.0);

  pnh_.param("guide_line_width_m", guide_line_width_m_, 0.10);
  pnh_.param("obb_width_tolerance_m", obb_width_tolerance_m_, 0.05);
  pnh_.param("obb_max_outlier_ratio", obb_max_outlier_ratio_, 0.30);
  pnh_.param("obb_min_ground_points", obb_min_ground_points_, 10);
  pnh_.param<std::string>("guide_line_estimator_type",
                          guide_line_estimator_type_,
                          "DepthGuideLineEstimator");
  pnh_.param<std::string>("ground_plane_provider_type",
                          ground_plane_provider_type_,
                          "topic");

  if (image_scale_ <= 0.0 || image_scale_ > 1.0)
  {
    ROS_WARN("image_scale must be in (0, 1], reset to 1.0");
    image_scale_ = 1.0;
  }
  if (roi_y_ratio_ < 0.0 || roi_y_ratio_ > 1.0)
  {
    ROS_WARN("roi_y_ratio must be in [0, 1], reset to 0.75");
    roi_y_ratio_ = 0.75;
  }
  if (roi_bottom_ratio_ < 0.0 || roi_bottom_ratio_ > 1.0)
  {
    ROS_WARN("roi_bottom_ratio must be in [0, 1], reset to 0.5");
    roi_bottom_ratio_ = 0.5;
  }

  if (guide_line_width_m_ <= 0.0)
  {
    ROS_WARN("guide_line_width_m must be > 0, reset to 0.10");
    guide_line_width_m_ = 0.10;
  }
  if (obb_width_tolerance_m_ < 0.0)
  {
    ROS_WARN("obb_width_tolerance_m must be >= 0, reset to 0.05");
    obb_width_tolerance_m_ = 0.05;
  }
  if (obb_max_outlier_ratio_ < 0.0 || obb_max_outlier_ratio_ > 1.0)
  {
    ROS_WARN("obb_max_outlier_ratio must be in [0, 1], reset to 0.30");
    obb_max_outlier_ratio_ = 0.30;
  }
  if (obb_min_ground_points_ < 2)
  {
    ROS_WARN("obb_min_ground_points must be >= 2, reset to 10");
    obb_min_ground_points_ = 10;
  }

  if (morph_kernel_size_ > 0 && morph_kernel_size_ % 2 == 0)
  {
    morph_kernel_size_ += 1;
    ROS_WARN("morph_kernel_size must be odd, adjusted to %d", morph_kernel_size_);
  }

  if (look_ahead_m_ < 0.0)
  {
    ROS_WARN("look_ahead_m must be >= 0, reset to 0.0");
    look_ahead_m_ = 0.0;
  }

  if (min_aspect_ratio_ < 1.0)
  {
    ROS_WARN("min_aspect_ratio must be >= 1.0, reset to 1.0");
    min_aspect_ratio_ = 1.0;
  }
  if (max_aspect_ratio_ > 0.0 && max_aspect_ratio_ < min_aspect_ratio_)
  {
    ROS_WARN("max_aspect_ratio (%.2f) < min_aspect_ratio (%.2f), disabling max filter",
             max_aspect_ratio_, min_aspect_ratio_);
    max_aspect_ratio_ = 0.0;
  }

  if (merge_max_angle_diff_deg_ < 0.0)
  {
    ROS_WARN("merge_max_angle_diff_deg must be >= 0, reset to 15.0");
    merge_max_angle_diff_deg_ = 15.0;
  }
  if (merge_max_region_gap_n_ < 0.0)
  {
    ROS_WARN("merge_max_region_gap_n must be >= 0, reset to 0.1");
    merge_max_region_gap_n_ = 0.1;
  }

  if (mask_filter_alpha_ < 0.0)
  {
    ROS_WARN("mask_filter_alpha must be >= 0, reset to 0.0");
    mask_filter_alpha_ = 0.0;
  }
  if (mask_filter_alpha_ > 1.0)
  {
    ROS_WARN("mask_filter_alpha must be <= 1.0, reset to 1.0");
    mask_filter_alpha_ = 1.0;
  }

  if (error_stats_window_ < 1)
  {
    ROS_WARN("error_stats_window must be >= 1, reset to 100");
    error_stats_window_ = 100;
  }

  if (guide_line_estimator_type_ != "FitLineGuideLineEstimator" &&
      guide_line_estimator_type_ != "DepthGuideLineEstimator" &&
      guide_line_estimator_type_ != "RectangleGuideLineEstimator")
  {
    ROS_WARN("Unknown guide_line_estimator_type '%s', using DepthGuideLineEstimator",
             guide_line_estimator_type_.c_str());
    guide_line_estimator_type_ = "DepthGuideLineEstimator";
  }

  if (ground_plane_provider_type_ != "topic" &&
      ground_plane_provider_type_ != "imu")
  {
    ROS_WARN("Unknown ground_plane_provider_type '%s', using topic",
             ground_plane_provider_type_.c_str());
    ground_plane_provider_type_ = "topic";
  }
}

void ColorRegionDetector::rgbCallback(const sensor_msgs::ImageConstPtr& rgb_msg)
{
  if (!first_rgb_received_)
  {
    ROS_INFO("First RGB frame received: %dx%d", rgb_msg->width, rgb_msg->height);
    first_rgb_received_ = true;
  }

  cv_bridge::CvImageConstPtr cv_rgb;
  try
  {
    cv_rgb = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  processFrame(rgb_msg, cv_rgb->image);
}

void ColorRegionDetector::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  if (camera_info_received_)
  {
    return;
  }

  const auto& k = info_msg->K;

  camera_intrinsics_.fx = static_cast<float>(k[0]);
  camera_intrinsics_.fy = static_cast<float>(k[4]);
  camera_intrinsics_.cx = static_cast<float>(k[2]);
  camera_intrinsics_.cy = static_cast<float>(k[5]);

  camera_info_received_ = true;

  guide_line_estimator_->configure(camera_intrinsics_, image_scale_, target_angle_,
                                   roi_y_ratio_, roi_y_, look_ahead_m_);

  ROS_INFO("CameraInfo received");
  ROS_INFO("  raw intrinsics: fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
           camera_intrinsics_.fx, camera_intrinsics_.fy,
           camera_intrinsics_.cx, camera_intrinsics_.cy);
  ROS_INFO("  image_scale:    %.2f", image_scale_);
  if (guide_line_estimator_type_ == "DepthGuideLineEstimator")
  {
    ROS_INFO("  Depth estimator will use scaled intrinsics: "
             "fx=%.3f fy=%.3f cx=%.3f cy=%.3f",
             camera_intrinsics_.fx * image_scale_,
             camera_intrinsics_.fy * image_scale_,
             camera_intrinsics_.cx * image_scale_,
             camera_intrinsics_.cy * image_scale_);
  }
}

void ColorRegionDetector::processFrame(const sensor_msgs::ImageConstPtr& rgb_msg,
                                       const cv::Mat& bgr_image)
{
  ++frame_counter_;

  ensureTargetAngleComputed();

  cv::Mat scaled_bgr;
  const cv::Mat* proc_image = &bgr_image;
  if (image_scale_ < 1.0)
  {
    cv::resize(bgr_image, scaled_bgr, cv::Size(), image_scale_, image_scale_, cv::INTER_LINEAR);
    proc_image = &scaled_bgr;
  }

  cv::Mat mask = createColorMask(*proc_image);

  // Temporal smoothing of the binary mask to reduce HSV segmentation jitter
  // for stationary cameras.  alpha=0 disables it.
  if (mask_filter_alpha_ > 0.0)
  {
    if (mask_accumulator_.empty() ||
        mask_accumulator_.size() != mask.size() ||
        mask_accumulator_.type() != CV_32F)
    {
      mask_accumulator_ = cv::Mat(mask.size(), CV_32F, cv::Scalar(0.0));
    }

    cv::Mat float_mask;
    mask.convertTo(float_mask, CV_32F, 1.0 / 255.0);
    cv::addWeighted(mask_accumulator_, 1.0 - mask_filter_alpha_,
                    float_mask, mask_filter_alpha_, 0.0,
                    mask_accumulator_);

    // Convert back to binary: accumulator > 0.5 -> 255, else 0.
    mask_accumulator_.convertTo(mask, CV_8U, 255.0);
    cv::threshold(mask, mask, 127, 255, cv::THRESH_BINARY);
  }

  applyMorphology(mask);

  std::vector<std::vector<cv::Point>> contours;
  detectContours(mask, contours);

  const double scaled_min_length = static_cast<double>(min_contour_length_) *
                                   image_scale_;

  // 1. Compute normalized features for all raw contours.
  const auto features = computeContourFeatures(contours, scaled_min_length);

  // 2. Collect valid contour indices.
  std::vector<size_t> valid_indices;
  valid_indices.reserve(features.size());
  for (size_t i = 0; i < features.size(); ++i)
  {
    if (features[i].valid)
    {
      valid_indices.push_back(i);
    }
  }

  // 3. Build merge groups sorted from bottom to top of the image.
  //    Each group is a candidate guide line; the bottom-most group is the nearest
  //    one to the camera and is tried first.
  const auto groups = findBottomUpMergeGroups(features, valid_indices);

  // 4. Draw all raw contours and build the 2D contour array.
  cv::Mat annotated = proc_image->clone();
  ContourArray contour_array;
  contour_array.header = rgb_msg->header;

  int id = 0;
  int filtered_count = 0;
  for (size_t i = 0; i < contours.size(); ++i)
  {
    ContourInfo info;
    if (!buildContourInfo(contours[i], id, info, scaled_min_length))
    {
      ++filtered_count;
      continue;
    }

    drawContourOnImage(annotated, contours[i], info, false);
    contour_array.contours.push_back(info);
    ++id;
  }

  double largest_length = 0.0;
  for (size_t idx : valid_indices)
  {
    const double length = cv::arcLength(contours[idx], true);
    if (length > largest_length)
    {
      largest_length = length;
    }
  }

  // 5. Try each group from bottom up until the estimator returns a valid result.
  GuideLineError error_msg;
  error_msg.header = rgb_msg->header;
  error_msg.valid = false;
  error_msg.yaw_error = 0.0;
  error_msg.lateral_error_n = 0.0;
  error_msg.lateral_error_m = 0.0;
  error_msg.roi_point.x = 0.0;
  error_msg.roi_point.y = 0.0;
  error_msg.roi_point.z = 0.0;

  std::string invalid_reason;
  int selected_group_idx = -1;
  if (!camera_intrinsics_.valid())
  {
    invalid_reason = "no valid camera_info";
  }
  else if (groups.empty())
  {
    invalid_reason = "no valid contour";
  }
  else
  {
    // Sort candidate groups so that the most reliable one is tried first.
    // We prioritize groups whose lowest point is in the bottom part of the
    // image (near the robot), and among those we pick the longest one.  If no
    // bottom group succeeds, we fall back to upper groups sorted by bottom-y.
    // The bottom region threshold is configurable via roi_bottom_ratio_.
    const double bottom_threshold = proc_image->rows * roi_bottom_ratio_;

    struct GroupScore
    {
      size_t index;
      double length;
      double bottom_y;
    };

    std::vector<GroupScore> scores;
    scores.reserve(groups.size());
    for (size_t g = 0; g < groups.size(); ++g)
    {
      double length = 0.0;
      double bottom_y = 0.0;
      for (size_t idx : groups[g])
      {
        length += cv::arcLength(contours[idx], true);
        const double b = features[idx].bbox.y + features[idx].bbox.height;
        if (b > bottom_y)
        {
          bottom_y = b;
        }
      }
      scores.push_back({ g, length, bottom_y });
    }

    std::sort(scores.begin(), scores.end(),
              [&](const GroupScore& a, const GroupScore& b)
              {
                const bool a_bottom = a.bottom_y >= bottom_threshold;
                const bool b_bottom = b.bottom_y >= bottom_threshold;
                if (a_bottom && b_bottom)
                {
                  return a.length > b.length;
                }
                if (a_bottom && !b_bottom)
                {
                  return true;
                }
                if (!a_bottom && b_bottom)
                {
                  return false;
                }
                return a.bottom_y > b.bottom_y;
              });

    for (size_t rank = 0; rank < scores.size(); ++rank)
    {
      const size_t g = scores[rank].index;
      const auto& group = groups[g];

      std::vector<cv::Point> merged;
      size_t total_points = 0;
      for (size_t idx : group)
      {
        total_points += contours[idx].size();
      }
      merged.reserve(total_points);
      for (size_t idx : group)
      {
        merged.insert(merged.end(), contours[idx].begin(), contours[idx].end());
      }

      const auto rect_contour = getRectangularContour(merged);

      // Use a temporary annotated image for candidate groups so that failed
      // groups do not clutter the final visualization.
      cv::Mat temp_annotated = annotated.clone();
      GuideLineError candidate = guide_line_estimator_->estimate(
          merged, rect_contour,
          proc_image->size(), rgb_msg->header, temp_annotated);

      if (candidate.valid)
      {
        error_msg = candidate;
        selected_group_idx = static_cast<int>(rank);
        annotated = std::move(temp_annotated);
        break;
      }
    }

    if (!error_msg.valid)
    {
      invalid_reason = "estimator";
      ROS_WARN_THROTTLE(1.0,
                        "GuideLineError invalid: no group produced valid result "
                        "(groups=%zu, see estimator logs)",
                        groups.size());
    }
  }

  if (!invalid_reason.empty() && selected_group_idx < 0)
  {
    ROS_WARN_THROTTLE(1.0,
                      "GuideLineError invalid: %s (raw contours: %zu, "
                      "groups: %zu, scaled_min_length: %.1f, image: %dx%d)",
                      invalid_reason.c_str(), contours.size(), groups.size(),
                      scaled_min_length, proc_image->cols, proc_image->rows);
  }

  guide_line_error_pub_.publish(error_msg);

  if (error_msg.valid)
  {
    updateErrorStats(error_msg);
    ROS_INFO_THROTTLE(1.0,
                      "[Frame %d] GuideLineError OK: "
                      "yaw=%.4f rad (%.1f deg), lat_n=%.4f, lat_m=%.4f, "
                      "roi=(%.1f, %.1f), selected_group=%d/%zu (members=%zu)",
                      frame_counter_,
                      error_msg.yaw_error,
                      error_msg.yaw_error * 180.0 / M_PI,
                      error_msg.lateral_error_n,
                      error_msg.lateral_error_m,
                      error_msg.roi_point.x,
                      error_msg.roi_point.y,
                      selected_group_idx + 1,
                      groups.size(),
                      groups[selected_group_idx].size());
  }
  else
  {
    ROS_WARN_THROTTLE(1.0,
                      "[Frame %d] GuideLineError INVALID: reason=%s, "
                      "camera_info=%s, groups=%zu, "
                      "raw contours=%zu, scaled_min_length=%.1f",
                      frame_counter_,
                      invalid_reason.c_str(),
                      camera_info_received_ ? "yes" : "no",
                      groups.size(),
                      contours.size(),
                      scaled_min_length);
  }

  ROS_DEBUG_THROTTLE(1.0,
                     "[Frame %d] Detection: image=%dx%d scale=%.2f, "
                     "raw_contours=%zu, groups=%zu, valid_2d=%zu, "
                     "filtered=%d, scaled_min_length=%.1f, largest_length=%.1f",
                     frame_counter_,
                     proc_image->cols, proc_image->rows, image_scale_,
                     contours.size(), groups.size(), contour_array.contours.size(),
                     filtered_count, scaled_min_length, largest_length);

  if (enable_error_stats_)
  {
    drawErrorStatsOverlay(annotated, error_msg);
  }

  publishResults(rgb_msg->header, mask, annotated, contour_array);
}

std::vector<cv::Point> ColorRegionDetector::getRectangularContour(
    const std::vector<cv::Point>& contour) const
{
  std::vector<cv::Point> rect;
  if (contour.size() < 3)
  {
    return rect;
  }

  const double area = cv::contourArea(contour);
  if (area <= 1e-6)
  {
    return rect;
  }

  try
  {
    std::vector<cv::Point2f> pts(contour.begin(), contour.end());
    cv::RotatedRect rotated_rect = cv::minAreaRect(pts);
    if (rotated_rect.size.width <= 0.0f || rotated_rect.size.height <= 0.0f)
    {
      return rect;
    }

    cv::Mat box_mat;
    cv::boxPoints(rotated_rect, box_mat);

    std::vector<cv::Point2f> box_float;
    box_float.reserve(4);
    for (int i = 0; i < box_mat.rows; ++i)
    {
      box_float.emplace_back(box_mat.at<float>(i, 0), box_mat.at<float>(i, 1));
    }

    cv::Point2f center = rotated_rect.center;
    std::sort(box_float.begin(), box_float.end(),
              [&center](const cv::Point2f& a, const cv::Point2f& b)
              {
                const float angle_a = std::atan2(a.y - center.y, a.x - center.x);
                const float angle_b = std::atan2(b.y - center.y, b.x - center.x);
                return angle_a < angle_b;
              });

    rect.reserve(box_float.size());
    for (const auto& p : box_float)
    {
      rect.emplace_back(static_cast<int>(std::round(p.x)), static_cast<int>(std::round(p.y)));
    }
  }
  catch (const cv::Exception& e)
  {
    ROS_WARN_THROTTLE(1.0, "getRectangularContour failed: %s", e.what());
    rect.clear();
  }

  return rect;
}

bool ColorRegionDetector::computeTargetAngleFromTf()
{
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform(camera_frame_, base_frame_,
                                           ros::Time(0), ros::Duration(0.5));
  }
  catch (const tf2::TransformException& ex)
  {
    ROS_WARN_THROTTLE(5.0,
                      "TF lookup from %s to %s failed: %s",
                      base_frame_.c_str(), camera_frame_.c_str(), ex.what());
    return false;
  }

  // Vehicle forward direction in base_link is +X.
  const Eigen::Vector3d forward_base(1.0, 0.0, 0.0);

  // Rotate into camera optical frame. Direction vectors only need rotation.
  const Eigen::Quaterniond q(transform.transform.rotation.w,
                             transform.transform.rotation.x,
                             transform.transform.rotation.y,
                             transform.transform.rotation.z);
  const Eigen::Vector3d forward_cam = q * forward_base;

  if (std::abs(forward_cam.z()) < 1e-6)
  {
    ROS_WARN_THROTTLE(5.0,
                      "Vehicle forward direction is parallel to the image plane");
    return false;
  }

  // Normalized image-plane projection. The angle is independent of fx/fy.
  const double x_n = forward_cam.x() / forward_cam.z();
  const double y_n = forward_cam.y() / forward_cam.z();

  target_angle_ = std::atan2(y_n, x_n);
  target_angle_computed_ = true;

  // Re-configure the estimator with the newly computed target angle.
  if (guide_line_estimator_ && camera_intrinsics_.valid())
  {
    guide_line_estimator_->configure(camera_intrinsics_, image_scale_, target_angle_,
                                     roi_y_ratio_, roi_y_, look_ahead_m_);
  }

  ROS_INFO("Computed target_angle from TF:");
  ROS_INFO("  target_angle: %.4f rad (%.1f deg)",
           target_angle_, target_angle_ * 180.0 / M_PI);
  ROS_INFO("  forward_cam in optical frame: (%.3f, %.3f, %.3f)",
           forward_cam.x(), forward_cam.y(), forward_cam.z());
  return true;
}

void ColorRegionDetector::ensureTargetAngleComputed()
{
  if (!use_tf_target_angle_ || target_angle_computed_)
  {
    return;
  }

  if (!computeTargetAngleFromTf())
  {
    ROS_WARN_THROTTLE(5.0,
                      "Failed to compute target_angle from TF. Using fallback "
                      "target_angle=%.4f rad from config.",
                      target_angle_);
  }
}

cv::Mat ColorRegionDetector::createColorMask(const cv::Mat& bgr_image)
{
  cv::Mat hsv;
  cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), mask);
  return mask;
}

void ColorRegionDetector::applyMorphology(cv::Mat& mask)
{
  if (morph_kernel_size_ <= 0)
  {
    return;
  }

  cv::Mat element = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(morph_kernel_size_, morph_kernel_size_));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);
}

void ColorRegionDetector::detectContours(const cv::Mat& mask,
                                         std::vector<std::vector<cv::Point>>& contours)
{
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
}

bool ColorRegionDetector::buildContourInfo(const std::vector<cv::Point>& contour,
                                           int id,
                                           ContourInfo& info,
                                           double min_contour_length)
{
  const double length = cv::arcLength(contour, true);
  if (length < min_contour_length)
  {
    return false;
  }

  cv::Moments m = cv::moments(contour);
  cv::Point2f center(0, 0);
  if (m.m00 > 0)
  {
    center.x = static_cast<float>(m.m10 / m.m00);
    center.y = static_cast<float>(m.m01 / m.m00);
  }

  cv::Rect bbox = cv::boundingRect(contour);

  if (!isAspectRatioValid(bbox, id))
  {
    return false;
  }

  const double area = cv::contourArea(contour);

  info.id = id;
  info.center.x = center.x;
  info.center.y = center.y;
  info.center.z = 0.0;
  info.area = area;
  info.bbox_tl.x = bbox.x;
  info.bbox_tl.y = bbox.y;
  info.bbox_tl.z = 0.0;
  info.bbox_br.x = bbox.x + bbox.width;
  info.bbox_br.y = bbox.y + bbox.height;
  info.bbox_br.z = 0.0;

  size_t step = 1;
  if (max_contour_points_ > 0 && contour.size() > static_cast<size_t>(max_contour_points_))
  {
    step = contour.size() / max_contour_points_;
    if (step < 1)
    {
      step = 1;
    }
  }

  info.points.clear();
  for (size_t i = 0; i < contour.size(); i += step)
  {
    geometry_msgs::Point32 p;
    p.x = contour[i].x;
    p.y = contour[i].y;
    p.z = 0.0f;
    info.points.push_back(p);
  }

  return true;
}

bool ColorRegionDetector::isAspectRatioValid(const cv::Rect& bbox, int id) const
{
  if (bbox.width <= 0 || bbox.height <= 0)
  {
    return true;
  }

  const double aspect_ratio =
      std::max(static_cast<double>(bbox.width),
               static_cast<double>(bbox.height)) /
      std::min(static_cast<double>(bbox.width),
               static_cast<double>(bbox.height));

  if (aspect_ratio < min_aspect_ratio_ ||
      (max_aspect_ratio_ > 0.0 && aspect_ratio > max_aspect_ratio_))
  {
    ROS_WARN_THROTTLE(1.0,
                      "Contour %d filtered by aspect ratio: %.2f "
                      "(allowed [%.2f, %.2f])",
                      id, aspect_ratio,
                      min_aspect_ratio_,
                      max_aspect_ratio_ > 0.0
                          ? max_aspect_ratio_
                          : std::numeric_limits<double>::infinity());
    return false;
  }

  return true;
}

std::vector<NormalizedContourFeatures> ColorRegionDetector::computeContourFeatures(
    const std::vector<std::vector<cv::Point>>& contours,
    double min_contour_length) const
{
  const double fx = camera_intrinsics_.fx * image_scale_;
  const double fy = camera_intrinsics_.fy * image_scale_;
  const double cx = camera_intrinsics_.cx * image_scale_;
  const double cy = camera_intrinsics_.cy * image_scale_;

  std::vector<NormalizedContourFeatures> features;
  features.reserve(contours.size());

  for (size_t i = 0; i < contours.size(); ++i)
  {
    const auto& contour = contours[i];
    NormalizedContourFeatures f;
    f.bbox = cv::boundingRect(contour);

    const double length = cv::arcLength(contour, true);
    if (length < min_contour_length || !isAspectRatioValid(f.bbox, static_cast<int>(i)))
    {
      features.push_back(f);
      continue;
    }

    cv::Moments m = cv::moments(contour);
    cv::Point2f center(0.0f, 0.0f);
    if (m.m00 > 0.0)
    {
      center.x = static_cast<float>(m.m10 / m.m00);
      center.y = static_cast<float>(m.m01 / m.m00);
    }

    f.center_n.x = static_cast<float>((center.x - cx) / fx);
    f.center_n.y = static_cast<float>((center.y - cy) / fy);

    std::vector<cv::Point2f> normalized_points;
    normalized_points.reserve(contour.size());
    for (const auto& p : contour)
    {
      normalized_points.emplace_back(
          static_cast<float>((p.x - cx) / fx),
          static_cast<float>((p.y - cy) / fy));
    }

    if (normalized_points.size() >= 2)
    {
      cv::Mat line;
      cv::fitLine(normalized_points, line, cv::DIST_L2, 0, 0.01, 0.01);
      const float vx = line.at<float>(0, 0);
      const float vy = line.at<float>(1, 0);
      f.angle = std::atan2(vy, vx);
    }

    f.valid = true;
    features.push_back(f);
  }

  return features;
}

std::vector<std::vector<size_t>> ColorRegionDetector::findBottomUpMergeGroups(
    const std::vector<NormalizedContourFeatures>& features,
    const std::vector<size_t>& valid_indices) const
{
  std::vector<std::vector<size_t>> groups;
  if (valid_indices.empty())
  {
    return groups;
  }

  if (enable_contour_merging_ && valid_indices.size() >= 2 &&
      camera_intrinsics_.valid())
  {
    const double angle_thresh_rad = merge_max_angle_diff_deg_ * M_PI / 180.0;

    UnionFind uf(valid_indices.size());

    for (size_t a = 0; a < valid_indices.size(); ++a)
    {
      const size_t idx_a = valid_indices[a];
      const auto& fa = features[idx_a];

      for (size_t b = a + 1; b < valid_indices.size(); ++b)
      {
        const size_t idx_b = valid_indices[b];
        const auto& fb = features[idx_b];

        const double angle_diff = lineAngleDifference(fa.angle, fb.angle);
        if (angle_diff > angle_thresh_rad)
        {
          continue;
        }

        const double region_gap = computeNormalizedBboxGap(fa.bbox, fb.bbox);
        if (region_gap > merge_max_region_gap_n_)
        {
          continue;
        }

        uf.unite(a, b);
      }
    }

    std::map<size_t, std::vector<size_t>> groups_map;
    for (size_t a = 0; a < valid_indices.size(); ++a)
    {
      groups_map[uf.find(a)].push_back(valid_indices[a]);
    }

    groups.reserve(groups_map.size());
    for (auto& kv : groups_map)
    {
      groups.push_back(std::move(kv.second));
    }
  }
  else
  {
    // Merging disabled: each valid contour is its own group.
    groups.reserve(valid_indices.size());
    for (size_t idx : valid_indices)
    {
      groups.push_back({ idx });
    }
  }

  // Sort groups by the bottom of their lowest bounding box (descending),
  // so the nearest group to the camera is tried first.
  std::sort(groups.begin(), groups.end(),
            [&](const std::vector<size_t>& a, const std::vector<size_t>& b)
            {
              auto max_bottom = [&](const std::vector<size_t>& group)
              {
                double bottom = 0.0;
                for (size_t idx : group)
                {
                  const double group_bottom =
                      features[idx].bbox.y + features[idx].bbox.height;
                  if (group_bottom > bottom)
                  {
                    bottom = group_bottom;
                  }
                }
                return bottom;
              };
              return max_bottom(a) > max_bottom(b);
            });

  return groups;
}

double ColorRegionDetector::computeNormalizedBboxGap(const cv::Rect& a,
                                                     const cv::Rect& b) const
{
  const double fx = camera_intrinsics_.fx * image_scale_;
  const double fy = camera_intrinsics_.fy * image_scale_;
  const double cx = camera_intrinsics_.cx * image_scale_;
  const double cy = camera_intrinsics_.cy * image_scale_;

  const double ax1 = (a.x - cx) / fx;
  const double ax2 = (a.x + a.width - cx) / fx;
  const double ay1 = (a.y - cy) / fy;
  const double ay2 = (a.y + a.height - cy) / fy;

  const double bx1 = (b.x - cx) / fx;
  const double bx2 = (b.x + b.width - cx) / fx;
  const double by1 = (b.y - cy) / fy;
  const double by2 = (b.y + b.height - cy) / fy;

  double gap_x = 0.0;
  if (ax2 < bx1)
  {
    gap_x = bx1 - ax2;
  }
  else if (bx2 < ax1)
  {
    gap_x = ax1 - bx2;
  }

  double gap_y = 0.0;
  if (ay2 < by1)
  {
    gap_y = by1 - ay2;
  }
  else if (by2 < ay1)
  {
    gap_y = ay1 - by2;
  }

  return std::sqrt(gap_x * gap_x + gap_y * gap_y);
}

void ColorRegionDetector::drawContourOnImage(cv::Mat& annotated,
                                             const std::vector<cv::Point>& contour,
                                             const ContourInfo& info,
                                             bool is_merged)
{
  cv::Rect bbox(
      static_cast<int>(info.bbox_tl.x),
      static_cast<int>(info.bbox_tl.y),
      static_cast<int>(info.bbox_br.x - info.bbox_tl.x),
      static_cast<int>(info.bbox_br.y - info.bbox_tl.y));

  cv::drawContours(annotated, std::vector<std::vector<cv::Point>>{contour}, -1,
                   cv::Scalar(0, 255, 0), 2);
  cv::circle(annotated,
             cv::Point(static_cast<int>(info.center.x), static_cast<int>(info.center.y)),
             4, cv::Scalar(0, 0, 255), -1);

  // Use magenta for merged contours so they are visually distinct.
  const cv::Scalar bbox_color = is_merged ? cv::Scalar(255, 0, 255)
                                          : cv::Scalar(255, 0, 0);
  cv::rectangle(annotated, bbox, bbox_color, 2);
}

void ColorRegionDetector::updateErrorStats(const GuideLineError& error)
{
  if (!error.valid)
  {
    return;
  }

  yaw_error_history_.push_back(error.yaw_error);
  if (yaw_error_history_.size() > static_cast<size_t>(error_stats_window_))
  {
    yaw_error_history_.pop_front();
  }

  // Use lateral_error_m when available (Depth / Rectangle mode), otherwise lateral_error_n.
  const double lateral = (guide_line_estimator_type_ == "DepthGuideLineEstimator" ||
                          guide_line_estimator_type_ == "RectangleGuideLineEstimator")
                             ? error.lateral_error_m
                             : error.lateral_error_n;
  lateral_error_history_.push_back(lateral);
  if (lateral_error_history_.size() > static_cast<size_t>(error_stats_window_))
  {
    lateral_error_history_.pop_front();
  }
}

void ColorRegionDetector::drawErrorStatsOverlay(cv::Mat& annotated,
                                                const GuideLineError& error) const
{
  if (annotated.empty())
  {
    return;
  }

  const int font = cv::FONT_HERSHEY_SIMPLEX;
  const double font_scale = 0.5;
  const int thickness = 1;
  const int line_height = static_cast<int>(20 * (font_scale / 0.5));
  const cv::Scalar text_color(0, 255, 0);  // Green
  const cv::Scalar bg_color(0, 0, 0);      // Black background
  const int margin = 10;
  const int x = margin;
  int y = margin + line_height;

  auto draw_line = [&](const std::string& text)
  {
    int baseline = 0;
    cv::Size text_size = cv::getTextSize(text, font, font_scale, thickness, &baseline);
    cv::rectangle(annotated,
                  cv::Point(x - 2, y - text_size.height - 2),
                  cv::Point(x + text_size.width + 4, y + 2),
                  bg_color, -1);
    cv::putText(annotated, text, cv::Point(x, y), font, font_scale,
                text_color, thickness, cv::LINE_AA);
    y += line_height;
  };

  const std::string lat_unit =
      (guide_line_estimator_type_ == "DepthGuideLineEstimator" ||
       guide_line_estimator_type_ == "RectangleGuideLineEstimator")
          ? "m"
          : "n";

  draw_line("Error statistics (window=" + std::to_string(error_stats_window_) + ")");

  if (error.valid)
  {
    draw_line(cv::format("yaw:  %.2f deg", error.yaw_error * 180.0 / M_PI));
  }
  else
  {
    draw_line("yaw:  --");
  }

  if (!yaw_error_history_.empty())
  {
    const double min_yaw = *std::min_element(yaw_error_history_.begin(),
                                             yaw_error_history_.end());
    const double max_yaw = *std::max_element(yaw_error_history_.begin(),
                                             yaw_error_history_.end());
    const double avg_yaw = std::accumulate(yaw_error_history_.begin(),
                                           yaw_error_history_.end(), 0.0) /
                           yaw_error_history_.size();
    draw_line(cv::format("yaw  min/max/avg: %.2f / %.2f / %.2f deg",
                         min_yaw * 180.0 / M_PI,
                         max_yaw * 180.0 / M_PI,
                         avg_yaw * 180.0 / M_PI));
  }
  else
  {
    draw_line("yaw  min/max/avg: -- / -- / --");
  }

  if (error.valid)
  {
    const double lateral =
        (guide_line_estimator_type_ == "DepthGuideLineEstimator" ||
         guide_line_estimator_type_ == "RectangleGuideLineEstimator")
            ? error.lateral_error_m
            : error.lateral_error_n;
    draw_line(cv::format("lat:  %.4f %s", lateral, lat_unit.c_str()));
  }
  else
  {
    draw_line("lat:  --");
  }

  if (!lateral_error_history_.empty())
  {
    const double min_lat = *std::min_element(lateral_error_history_.begin(),
                                             lateral_error_history_.end());
    const double max_lat = *std::max_element(lateral_error_history_.begin(),
                                             lateral_error_history_.end());
    const double avg_lat = std::accumulate(lateral_error_history_.begin(),
                                           lateral_error_history_.end(), 0.0) /
                           lateral_error_history_.size();
    draw_line(cv::format("lat  min/max/avg: %.4f / %.4f / %.4f %s",
                         min_lat, max_lat, avg_lat, lat_unit.c_str()));
  }
  else
  {
    draw_line("lat  min/max/avg: -- / -- / --");
  }
}

void ColorRegionDetector::publishResults(const std_msgs::Header& header,
                                         const cv::Mat& mask,
                                         const cv::Mat& annotated,
                                         const ContourArray& contour_array)
{
  ROS_DEBUG_THROTTLE(1.0,
                    "Publishing: mask + annotated + %zu 2D contours",
                    contour_array.contours.size());

  cv_bridge::CvImage mask_msg;
  mask_msg.header = header;
  mask_msg.encoding = sensor_msgs::image_encodings::MONO8;
  mask_msg.image = mask;
  mask_pub_.publish(mask_msg.toImageMsg());

  cv_bridge::CvImage annotated_msg;
  annotated_msg.header = header;
  annotated_msg.encoding = sensor_msgs::image_encodings::BGR8;
  annotated_msg.image = annotated;
  annotated_pub_.publish(annotated_msg.toImageMsg());

  contours_pub_.publish(contour_array);
}

}  // namespace gemini_geometry_detector
