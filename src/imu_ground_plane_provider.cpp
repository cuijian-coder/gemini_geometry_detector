#include "gemini_geometry_detector/imu_ground_plane_provider.h"

#include <ros/ros.h>

namespace gemini_geometry_detector
{

ImuGroundPlaneProvider::ImuGroundPlaneProvider()
  : imu_topic_("/camera/gyro_accel/sample")
  , camera_height_(0.8)
  , gravity_filter_alpha_(1.0)
  , q_imu_to_camera_(Eigen::Quaterniond::Identity())
  , gravity_filtered_(Eigen::Vector3d::Zero())
  , has_first_imu_(false)
{
}

void ImuGroundPlaneProvider::configure(ros::NodeHandle& pnh)
{
  pnh.param<std::string>("imu_topic", imu_topic_, "/camera/gyro_accel/sample");
  pnh.param<double>("camera_height", camera_height_, 0.8);
  pnh.param<double>("gravity_filter_alpha", gravity_filter_alpha_, 1.0);

  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
  pnh.param<double>("imu_to_camera_qx", qx, 0.0);
  pnh.param<double>("imu_to_camera_qy", qy, 0.0);
  pnh.param<double>("imu_to_camera_qz", qz, 0.0);
  pnh.param<double>("imu_to_camera_qw", qw, 1.0);

  q_imu_to_camera_ = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
  if (std::abs(q_imu_to_camera_.norm() - 1.0) > 1e-6)
  {
    ROS_WARN("[ImuGroundPlaneProvider] imu_to_camera quaternion auto-normalized");
  }
}

void ImuGroundPlaneProvider::start(ros::NodeHandle& nh, const PlaneCallback& callback)
{
  callback_ = callback;
  sub_ = nh.subscribe(imu_topic_, 10,
                      &ImuGroundPlaneProvider::imuCallback, this);

  ROS_INFO("[ImuGroundPlaneProvider] started");
  ROS_INFO("  imu_topic:            %s", imu_topic_.c_str());
  ROS_INFO("  camera_height:        %.4f m", camera_height_);
  ROS_INFO("  gravity_filter_alpha: %.4f", gravity_filter_alpha_);
  ROS_INFO("  imu_to_camera_q:      [%.4f, %.4f, %.4f, %.4f]",
           q_imu_to_camera_.x(), q_imu_to_camera_.y(),
           q_imu_to_camera_.z(), q_imu_to_camera_.w());
}

void ImuGroundPlaneProvider::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  if (!has_first_imu_)
  {
    ROS_INFO("[ImuGroundPlaneProvider] first IMU received, frame_id=%s, stamp=%.3f",
             msg->header.frame_id.c_str(),
             msg->header.stamp.toSec());
  }

  // sensor_msgs/Imu.linear_acceleration is the accelerometer reading.
  // For a static IMU it points opposite to gravity (upward), so actual
  // gravity = -linear_acceleration.
  Eigen::Vector3d gravity(-msg->linear_acceleration.x,
                          -msg->linear_acceleration.y,
                          -msg->linear_acceleration.z);

  if (gravity.norm() < kMinGravityNorm)
  {
    ROS_WARN_THROTTLE(1.0,
                      "[ImuGroundPlaneProvider] gravity norm too small (%.6f), skipping",
                      gravity.norm());
    return;
  }

  // IMU -> camera rotation compensation.
  if (!q_imu_to_camera_.isApprox(Eigen::Quaterniond::Identity()))
  {
    gravity = q_imu_to_camera_ * gravity;
  }

  // Low-pass filter.
  if (!has_first_imu_)
  {
    gravity_filtered_ = gravity;
    has_first_imu_ = true;
  }
  else
  {
    gravity_filtered_ = gravity_filter_alpha_ * gravity
                        + (1.0 - gravity_filter_alpha_) * gravity_filtered_;
  }

  const Eigen::Vector3d n = -gravity_filtered_.normalized();

  if (callback_)
  {
    callback_(Eigen::Vector3f(static_cast<float>(n.x()),
                              static_cast<float>(n.y()),
                              static_cast<float>(n.z())),
              static_cast<float>(camera_height_));
  }

  ROS_INFO_THROTTLE(5.0,
                    "[ImuGroundPlaneProvider] estimated plane: n=(%.3f, %.3f, %.3f), d=%.3f",
                    n.x(), n.y(), n.z(), camera_height_);
}

}  // namespace gemini_geometry_detector
