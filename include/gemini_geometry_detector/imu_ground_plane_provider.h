#ifndef GEMINI_GEOMETRY_DETECTOR_IMU_GROUND_PLANE_PROVIDER_H
#define GEMINI_GEOMETRY_DETECTOR_IMU_GROUND_PLANE_PROVIDER_H

#include <std_srvs/Trigger.h>

#include "gemini_geometry_detector/ground_plane_provider_interface.h"
#include "gemini_geometry_detector/imu_gravity_estimator.h"

namespace gemini_geometry_detector
{

/**
 * @brief Ground-plane provider that estimates the plane from IMU gravity.
 *
 * The heavy lifting of IMU sampling/filtering is delegated to
 * ImuGravityEstimator.  This class simply converts the estimated gravity
 * direction into a ground-plane (normal, d) and forwards it to the estimator.
 *
 * A ROS service is provided to trigger re-sampling of the IMU gravity
 * direction at runtime without restarting the node.
 */
class ImuGroundPlaneProvider : public IGroundPlaneProvider
{
public:
  ImuGroundPlaneProvider();
  ~ImuGroundPlaneProvider() override = default;

  void configure(ros::NodeHandle& pnh) override;
  void start(ros::NodeHandle& nh, const PlaneCallback& callback) override;

private:
  void onGravityUpdated(const Eigen::Vector3d& gravity_camera);
  bool resampleCallback(std_srvs::Trigger::Request& req,
                        std_srvs::Trigger::Response& resp);

  ImuGravityEstimator gravity_estimator_;
  PlaneCallback plane_callback_;
  double camera_height_ = 0.8;

  ros::ServiceServer resample_srv_;
  bool enable_resample_service_ = true;
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_IMU_GROUND_PLANE_PROVIDER_H
