#ifndef GEMINI_GEOMETRY_DETECTOR_COLOR_REGION_DETECTOR_H
#define GEMINI_GEOMETRY_DETECTOR_COLOR_REGION_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
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
  void processLatestImage();

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
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_COLOR_REGION_DETECTOR_H
