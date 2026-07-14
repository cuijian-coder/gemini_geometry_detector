#include "gemini_geometry_detector/topic_ground_plane_provider.h"

namespace gemini_geometry_detector
{

void TopicGroundPlaneProvider::configure(ros::NodeHandle& pnh)
{
  pnh.param<std::string>("ground_plane_topic", topic_name_, "ground_plane/coefficients");
  ROS_INFO("[TopicGroundPlaneProvider] configured:");
  ROS_INFO("  ground_plane_topic: %s", topic_name_.c_str());
}

void TopicGroundPlaneProvider::start(ros::NodeHandle& nh, const PlaneCallback& callback)
{
  callback_ = callback;
  sub_ = nh.subscribe(topic_name_, 1,
                      &TopicGroundPlaneProvider::groundPlaneCallback, this);

  ROS_INFO("[TopicGroundPlaneProvider] started, subscribed to %s", topic_name_.c_str());
}

void TopicGroundPlaneProvider::groundPlaneCallback(
    const ground_plane_calibrator::PlaneCoefficientsConstPtr& msg)
{
  static bool first = true;
  if (first)
  {
    ROS_INFO("[TopicGroundPlaneProvider] first ground plane received, "
             "frame_id=%s, stamp=%.3f",
             msg->header.frame_id.c_str(),
             msg->header.stamp.toSec());
    first = false;
  }

  Eigen::Vector3f normal;
  normal.x() = static_cast<float>(msg->A);
  normal.y() = static_cast<float>(msg->B);
  normal.z() = static_cast<float>(msg->C);

  if (callback_)
  {
    callback_(normal, static_cast<float>(msg->D));
  }

  ROS_INFO_THROTTLE(5.0,
                    "[TopicGroundPlaneProvider] received plane: "
                    "frame_id=%s, n=(%.3f, %.3f, %.3f), d=%.3f",
                    msg->header.frame_id.c_str(),
                    msg->A, msg->B, msg->C, msg->D);
}

}  // namespace gemini_geometry_detector
