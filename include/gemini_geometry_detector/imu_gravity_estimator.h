#ifndef GEMINI_GEOMETRY_DETECTOR_IMU_GRAVITY_ESTIMATOR_H
#define GEMINI_GEOMETRY_DETECTOR_IMU_GRAVITY_ESTIMATOR_H

#include <functional>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace gemini_geometry_detector
{

/**
 * @brief Estimates the gravity direction in the camera frame from IMU readings.
 *
 * Supports two modes:
 *   - Continuous: every valid IMU message updates a low-pass filtered gravity
 *     estimate and triggers the callback.
 *   - Sample-averaging: collects the first N valid IMU messages, averages them,
 *     triggers the callback once, and then optionally stops subscribing.
 *
 * The output gravity vector is in the camera frame (after applying the IMU→camera
 * rotation) and points opposite to the accelerometer reading (i.e. downward).
 */
class ImuGravityEstimator
{
public:
  using GravityCallback = std::function<void(const Eigen::Vector3d& gravity_camera)>;

  ImuGravityEstimator();
  ~ImuGravityEstimator() = default;

  /**
   * @brief Configure the estimator.
   *
   * @param imu_topic           ROS topic to subscribe to.
   * @param q_imu_to_camera     Rotation from IMU frame to camera frame.
   * @param gravity_filter_alpha Low-pass filter coefficient for continuous mode.
   *                             1.0 means no history averaging; 0.0 means keep first.
   * @param sample_count        Number of valid IMU messages to average before
   *                            considering the estimate complete.
   *                            <= 0 means continuous mode (never stop).
   */
  void configure(const std::string& imu_topic,
                 const Eigen::Quaterniond& q_imu_to_camera,
                 double gravity_filter_alpha,
                 int sample_count);

  /**
   * @brief Start subscribing to the IMU topic.
   */
  void start(ros::NodeHandle& nh, const GravityCallback& callback);

  /**
   * @brief Stop subscribing to the IMU topic.
   */
  void stop();

  /**
   * @brief Reset internal state and re-subscribe to collect a new sample set.
   *
   * In continuous mode this has no effect except re-arming the first-sample flag.
   */
  void resetAndResample();

  /**
   * @brief Whether a valid gravity estimate has been produced.
   */
  bool hasGravity() const;

  /**
   * @brief Return the latest normalized gravity direction in the camera frame.
   *        Returns zero vector if hasGravity() is false.
   */
  Eigen::Vector3d getGravity() const;

private:
  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
  void computeAndPublishAverage();

  std::string imu_topic_;
  Eigen::Quaterniond q_imu_to_camera_;
  double gravity_filter_alpha_ = 1.0;
  int sample_count_ = 0;  // <=0 = continuous mode.

  ros::NodeHandle* nh_ = nullptr;
  ros::Subscriber sub_;
  GravityCallback callback_;

  bool has_first_imu_ = false;
  Eigen::Vector3d gravity_filtered_ = Eigen::Vector3d::Zero();
  std::vector<Eigen::Vector3d> gravity_samples_;
  bool sampling_complete_ = false;

  static constexpr double kMinGravityNorm = 1e-3;
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_IMU_GRAVITY_ESTIMATOR_H
