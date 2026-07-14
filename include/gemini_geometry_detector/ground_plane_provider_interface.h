#ifndef GEMINI_GEOMETRY_DETECTOR_GROUND_PLANE_PROVIDER_INTERFACE_H
#define GEMINI_GEOMETRY_DETECTOR_GROUND_PLANE_PROVIDER_INTERFACE_H

#include <functional>
#include <ros/ros.h>
#include <Eigen/Core>

namespace gemini_geometry_detector
{

/**
 * @brief Interface for ground-plane coefficient sources.
 *
 * Implementations are responsible for obtaining the ground plane equation
 * (normal·P + d = 0) and notifying the consumer via the configured callback.
 */
class IGroundPlaneProvider
{
public:
  using PlaneCallback = std::function<void(const Eigen::Vector3f& normal, float d)>;

  virtual ~IGroundPlaneProvider() = default;

  /**
   * @brief Load provider-specific parameters.
   */
  virtual void configure(ros::NodeHandle& pnh) = 0;

  /**
   * @brief Start the provider.
   *
   * The provider will call @p callback whenever a new ground plane is available.
   */
  virtual void start(ros::NodeHandle& nh, const PlaneCallback& callback) = 0;
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_GROUND_PLANE_PROVIDER_INTERFACE_H
