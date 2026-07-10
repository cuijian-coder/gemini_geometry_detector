#ifndef GEMINI_GEOMETRY_DETECTOR_COLOR_REGION_DETECTOR_H
#define GEMINI_GEOMETRY_DETECTOR_COLOR_REGION_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <gemini_geometry_detector/ContourInfo.h>
#include <gemini_geometry_detector/ContourArray.h>
#include <mutex>

namespace gemini_geometry_detector
{

class ColorRegionDetector
{
public:
  explicit ColorRegionDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~ColorRegionDetector() = default;

private:
  void loadParameters();
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);
  void timerCallback(const ros::TimerEvent& event);

  // Main processing pipeline
  void processLatestImage();

  // Step 1: fetch the latest image safely
  sensor_msgs::ImageConstPtr getLatestImage();

  // Step 2: convert ROS image to OpenCV
  cv_bridge::CvImageConstPtr convertToCvImage(const sensor_msgs::ImageConstPtr& image_msg);

  // Step 3: create HSV binary mask
  cv::Mat createColorMask(const cv::Mat& bgr_image);

  // Step 4: apply morphological operations
  void applyMorphology(cv::Mat& mask);

  // Step 5: detect external contours
  void detectContours(const cv::Mat& mask, std::vector<std::vector<cv::Point>>& contours);

  // Step 6: build ContourInfo message from OpenCV contour; returns false if filtered out
  bool buildContourInfo(const std::vector<cv::Point>& contour, int id, ContourInfo& info);

  // Step 7: draw a single contour on the annotated image
  void drawContourOnImage(cv::Mat& annotated, const std::vector<cv::Point>& contour, const ContourInfo& info);

  // Step 8: publish mask, annotated image and contour array
  void publishResults(const std_msgs::Header& header,
                      const cv::Mat& mask,
                      const cv::Mat& annotated,
                      const ContourArray& contour_array);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;

  // Subscribers & Publishers
  image_transport::Subscriber image_sub_;
  image_transport::Publisher mask_pub_;
  image_transport::Publisher annotated_pub_;
  ros::Publisher contours_pub_;
  ros::Timer process_timer_;

  // Parameters
  std::string input_topic_;
  int h_min_, h_max_;
  int s_min_, s_max_;
  int v_min_, v_max_;
  int morph_kernel_size_;
  int min_contour_area_;
  int max_contour_points_;
  double process_rate_;

  // State
  sensor_msgs::ImageConstPtr latest_image_;
  std::mutex image_mutex_;
  bool has_image_;
  bool first_image_received_;
  bool first_image_processed_;
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_COLOR_REGION_DETECTOR_H
