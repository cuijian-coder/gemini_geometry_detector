#ifndef GEMINI_GEOMETRY_DETECTOR_TOPIC_GROUND_PLANE_PROVIDER_H
#define GEMINI_GEOMETRY_DETECTOR_TOPIC_GROUND_PLANE_PROVIDER_H

#include "gemini_geometry_detector/ground_plane_provider_interface.h"
#include "ground_plane_calibrator/PlaneCoefficients.h"

namespace gemini_geometry_detector
{

/**
 * @brief Ground-plane provider that listens to a PlaneCoefficients topic.
 */
class TopicGroundPlaneProvider : public IGroundPlaneProvider
{
public:
  TopicGroundPlaneProvider() = default;
  ~TopicGroundPlaneProvider() override = default;

  void configure(ros::NodeHandle& pnh) override;
  void start(ros::NodeHandle& nh, const PlaneCallback& callback) override;

private:
  void groundPlaneCallback(const ground_plane_calibrator::PlaneCoefficientsConstPtr& msg);

  ros::Subscriber sub_;
  PlaneCallback callback_;
  std::string topic_name_;
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_TOPIC_GROUND_PLANE_PROVIDER_H
