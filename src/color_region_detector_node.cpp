#include <ros/ros.h>
#include "gemini_geometry_detector/color_region_detector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "color_region_detector_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  gemini_geometry_detector::ColorRegionDetector detector(nh, pnh);

  ros::spin();
  return 0;
}
