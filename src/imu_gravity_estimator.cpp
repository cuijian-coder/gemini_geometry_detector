#include "gemini_geometry_detector/imu_gravity_estimator.h"

#include <ros/ros.h>

namespace gemini_geometry_detector
{

ImuGravityEstimator::ImuGravityEstimator()
  : imu_topic_("/camera/gyro_accel/sample")
  , q_imu_to_camera_(Eigen::Quaterniond::Identity())
  , gravity_filter_alpha_(1.0)
  , sample_count_(0)
  , has_first_imu_(false)
  , gravity_filtered_(Eigen::Vector3d::Zero())
  , sampling_complete_(false)
{
}

void ImuGravityEstimator::configure(const std::string& imu_topic,
                                    const Eigen::Quaterniond& q_imu_to_camera,
                                    double gravity_filter_alpha,
                                    int sample_count)
{
  imu_topic_ = imu_topic;
  q_imu_to_camera_ = q_imu_to_camera;
  gravity_filter_alpha_ = gravity_filter_alpha;
  sample_count_ = sample_count;
}

void ImuGravityEstimator::start(ros::NodeHandle& nh, const GravityCallback& callback)
{
  nh_ = &nh;
  callback_ = callback;
  sub_ = nh.subscribe(imu_topic_, 10,
                      &ImuGravityEstimator::imuCallback, this);

  ROS_INFO("[ImuGravityEstimator] started");
  ROS_INFO("  imu_topic:            %s", imu_topic_.c_str());
  ROS_INFO("  gravity_filter_alpha: %.4f", gravity_filter_alpha_);
  ROS_INFO("  sample_count:         %d (%s)",
           sample_count_,
           sample_count_ > 0 ? "sample-average mode" : "continuous mode");
}

void ImuGravityEstimator::stop()
{
  sub_.shutdown();
  ROS_INFO("[ImuGravityEstimator] stopped");
}

void ImuGravityEstimator::resetAndResample()
{
  has_first_imu_ = false;
  gravity_filtered_ = Eigen::Vector3d::Zero();
  gravity_samples_.clear();
  sampling_complete_ = false;

  if (!nh_)
  {
    ROS_WARN("[ImuGravityEstimator] resetAndResample() called before start()");
    return;
  }

  if (!sub_)
  {
    ROS_INFO("[ImuGravityEstimator] re-subscribing for resampling");
    sub_ = nh_->subscribe(imu_topic_, 10,
                          &ImuGravityEstimator::imuCallback, this);
  }
  else
  {
    ROS_INFO("[ImuGravityEstimator] reset for resampling");
  }
}

bool ImuGravityEstimator::hasGravity() const
{
  return has_first_imu_;
}

Eigen::Vector3d ImuGravityEstimator::getGravity() const
{
  if (!has_first_imu_)
  {
    return Eigen::Vector3d::Zero();
  }

  if (sample_count_ > 0 && sampling_complete_ && !gravity_samples_.empty())
  {
    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    for (const auto& g : gravity_samples_)
    {
      sum += g;
    }
    return sum.normalized();
  }

  return gravity_filtered_.normalized();
}

void ImuGravityEstimator::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  if (!has_first_imu_)
  {
    ROS_INFO("[ImuGravityEstimator] first IMU received, frame_id=%s, stamp=%.3f",
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
                      "[ImuGravityEstimator] gravity norm too small (%.6f), skipping",
                      gravity.norm());
    return;
  }

  // IMU -> camera rotation compensation.
  if (!q_imu_to_camera_.isApprox(Eigen::Quaterniond::Identity()))
  {
    gravity = q_imu_to_camera_ * gravity;
  }

  if (sample_count_ > 0)
  {
    // Sample-averaging mode.
    if (sampling_complete_)
    {
      return;
    }

    gravity_samples_.push_back(gravity);
    ROS_INFO_THROTTLE(0.5,
                      "[ImuGravityEstimator] collected %zu / %d samples",
                      gravity_samples_.size(), sample_count_);

    if (static_cast<int>(gravity_samples_.size()) >= sample_count_)
    {
      computeAndPublishAverage();
    }
  }
  else
  {
    // Continuous mode.
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

    if (callback_)
    {
      callback_(gravity_filtered_.normalized());
    }
  }
}

void ImuGravityEstimator::computeAndPublishAverage()
{
  if (gravity_samples_.empty())
  {
    return;
  }

  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  for (const auto& g : gravity_samples_)
  {
    sum += g;
  }

  gravity_filtered_ = sum;
  has_first_imu_ = true;
  sampling_complete_ = true;

  ROS_INFO("[ImuGravityEstimator] sampling complete: %zu samples, "
           "gravity=(%.3f, %.3f, %.3f), norm=%.3f",
           gravity_samples_.size(),
           gravity_filtered_.x(), gravity_filtered_.y(),
           gravity_filtered_.z(), gravity_filtered_.norm());

  if (callback_)
  {
    callback_(gravity_filtered_.normalized());
  }

  // In sample-average mode we stop after the first successful batch.
  stop();
}

}  // namespace gemini_geometry_detector
