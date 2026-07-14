#ifndef GEMINI_GEOMETRY_DETECTOR_IMU_GROUND_PLANE_PROVIDER_H
#define GEMINI_GEOMETRY_DETECTOR_IMU_GROUND_PLANE_PROVIDER_H

#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gemini_geometry_detector/ground_plane_provider_interface.h"

namespace gemini_geometry_detector
{

/**
 * @brief Ground-plane provider that estimates the plane from IMU gravity.
 *
 * The ground normal is computed as the opposite of the (filtered) gravity
 * vector rotated into the camera frame. The plane offset d is set to the
 * configured camera height.
 */
class ImuGroundPlaneProvider : public IGroundPlaneProvider
{
public:
  ImuGroundPlaneProvider();
  ~ImuGroundPlaneProvider() override = default;

  void configure(ros::NodeHandle& pnh) override;
  void start(ros::NodeHandle& nh, const PlaneCallback& callback) override;

private:
  void imuCallback(const sensor_msgs::ImuConstPtr& msg);

  ros::Subscriber sub_;
  PlaneCallback callback_;

  std::string imu_topic_;
  double camera_height_;
  double gravity_filter_alpha_;
  Eigen::Quaterniond q_imu_to_camera_;

  Eigen::Vector3d gravity_filtered_;
  bool has_first_imu_;

  static constexpr double kMinGravityNorm = 1e-3;
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_IMU_GROUND_PLANE_PROVIDER_H
