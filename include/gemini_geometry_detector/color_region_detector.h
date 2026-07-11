#ifndef GEMINI_GEOMETRY_DETECTOR_COLOR_REGION_DETECTOR_H
#define GEMINI_GEOMETRY_DETECTOR_COLOR_REGION_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

#include "gemini_geometry_detector/ContourInfo.h"
#include "gemini_geometry_detector/ContourArray.h"
#include "gemini_geometry_detector/depth_projector.h"
#include "gemini_geometry_detector/hybrid_ground_segmenter.h"

namespace gemini_geometry_detector
{

class ColorRegionDetector
{
public:
  explicit ColorRegionDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~ColorRegionDetector() = default;

private:
  void loadParameters();

  void rgbDepthInfoCallback(const sensor_msgs::ImageConstPtr& rgb_msg,
                            const sensor_msgs::ImageConstPtr& depth_msg,
                            const sensor_msgs::CameraInfoConstPtr& info_msg);

  void rgbCloudCallback(const sensor_msgs::ImageConstPtr& rgb_msg,
                        const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

  void processFrame(const sensor_msgs::ImageConstPtr& rgb_msg,
                    const cv::Mat& bgr_image,
                    const sensor_msgs::ImageConstPtr& depth_msg,
                    const sensor_msgs::CameraInfoConstPtr& info_msg);

  void processFrameCloud(const sensor_msgs::ImageConstPtr& rgb_msg,
                         const cv::Mat& bgr_image,
                         const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

  cv::Mat createColorMask(const cv::Mat& bgr_image);
  void applyMorphology(cv::Mat& mask);
  void detectContours(const cv::Mat& mask, std::vector<std::vector<cv::Point>>& contours);
  bool buildContourInfo(const std::vector<cv::Point>& contour, int id, ContourInfo& info);
  bool buildContourInfo3DFromCloud(const std::vector<cv::Point>& contour,
                                   int id,
                                   ContourInfo& info,
                                   const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void drawContourOnImage(cv::Mat& annotated, const std::vector<cv::Point>& contour, const ContourInfo& info);

  void publishResults(const std_msgs::Header& header,
                      const cv::Mat& mask,
                      const cv::Mat& annotated,
                      const ContourArray& contour_array);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;

  image_transport::SubscriberFilter rgb_sub_filter_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
  using CloudSyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::PointCloud2>;
  std::shared_ptr<message_filters::Synchronizer<CloudSyncPolicy>> cloud_sync_;

  image_transport::Publisher mask_pub_;
  image_transport::Publisher annotated_pub_;
  ros::Publisher contours_pub_;
  ros::Publisher ground_cloud_pub_;
  ros::Publisher depth_cloud_pub_;

  DepthProjector depth_projector_;
  HybridGroundSegmenter ground_segmenter_;

  std::string input_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string point_cloud_topic_;

  bool use_point_cloud_;

  int h_min_, h_max_;
  int s_min_, s_max_;
  int v_min_, v_max_;
  int morph_kernel_size_;
  int min_contour_area_;
  int max_contour_points_;
  double depth_scale_;

  bool first_sync_received_;
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_COLOR_REGION_DETECTOR_H
