#include "gemini_geometry_detector/imu_ground_plane_provider.h"

#include <ros/ros.h>

namespace gemini_geometry_detector
{

ImuGroundPlaneProvider::ImuGroundPlaneProvider()
  : camera_height_(0.8)
  , enable_resample_service_(true)
{
}

void ImuGroundPlaneProvider::configure(ros::NodeHandle& pnh)
{
  std::string imu_topic = "/camera/gyro_accel/sample";
  double camera_height = 0.8;
  double gravity_filter_alpha = 1.0;
  int sample_count = 0;

  pnh.param<std::string>("imu_topic", imu_topic, "/camera/gyro_accel/sample");
  pnh.param<double>("camera_height", camera_height, 0.8);
  pnh.param<double>("gravity_filter_alpha", gravity_filter_alpha, 1.0);
  pnh.param<int>("imu_sample_count", sample_count, 0);
  pnh.param<bool>("imu_enable_resample_service", enable_resample_service_, true);

  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
  pnh.param<double>("imu_to_camera_qx", qx, 0.0);
  pnh.param<double>("imu_to_camera_qy", qy, 0.0);
  pnh.param<double>("imu_to_camera_qz", qz, 0.0);
  pnh.param<double>("imu_to_camera_qw", qw, 1.0);

  Eigen::Quaterniond q_imu_to_camera(qw, qx, qy, qz);
  if (std::abs(q_imu_to_camera.norm() - 1.0) > 1e-6)
  {
    ROS_WARN("[ImuGroundPlaneProvider] imu_to_camera quaternion auto-normalized");
    q_imu_to_camera.normalize();
  }

  camera_height_ = camera_height;
  gravity_estimator_.configure(imu_topic, q_imu_to_camera,
                               gravity_filter_alpha, sample_count);
}

void ImuGroundPlaneProvider::start(ros::NodeHandle& nh, const PlaneCallback& callback)
{
  plane_callback_ = callback;

  gravity_estimator_.start(nh,
                           [this](const Eigen::Vector3d& gravity_camera)
                           {
                             this->onGravityUpdated(gravity_camera);
                           });

  if (enable_resample_service_)
  {
    resample_srv_ = nh.advertiseService(
        "resample_imu_ground_plane",
        &ImuGroundPlaneProvider::resampleCallback, this);
    ROS_INFO("[ImuGroundPlaneProvider] service advertised: resample_imu_ground_plane");
  }
}

void ImuGroundPlaneProvider::onGravityUpdated(const Eigen::Vector3d& gravity_camera)
{
  // gravity_camera points downward. Ground normal points away from the ground,
  // i.e. opposite to gravity.
  const Eigen::Vector3d n = -gravity_camera.normalized();

  if (plane_callback_)
  {
    plane_callback_(Eigen::Vector3f(static_cast<float>(n.x()),
                                    static_cast<float>(n.y()),
                                    static_cast<float>(n.z())),
                    static_cast<float>(camera_height_));
  }

  ROS_INFO_THROTTLE(5.0,
                    "[ImuGroundPlaneProvider] estimated plane: n=(%.3f, %.3f, %.3f), d=%.3f",
                    n.x(), n.y(), n.z(), camera_height_);
}

bool ImuGroundPlaneProvider::resampleCallback(std_srvs::Trigger::Request& /*req*/,
                                              std_srvs::Trigger::Response& resp)
{
  ROS_INFO("[ImuGroundPlaneProvider] resample requested");
  gravity_estimator_.resetAndResample();

  resp.success = true;
  resp.message = "IMU ground plane resampling started";
  return true;
}

}  // namespace gemini_geometry_detector
