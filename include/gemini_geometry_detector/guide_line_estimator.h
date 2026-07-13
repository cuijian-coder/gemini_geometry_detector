#ifndef GEMINI_GEOMETRY_DETECTOR_GUIDE_LINE_ESTIMATOR_H
#define GEMINI_GEOMETRY_DETECTOR_GUIDE_LINE_ESTIMATOR_H

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <mutex>
#include <vector>

#include "gemini_geometry_detector/Contour3DInfo.h"
#include "gemini_geometry_detector/Contour3DArray.h"

namespace gemini_geometry_detector
{

/**
 * @brief Estimates a 3D guide line on the ground plane from a 2D color contour.
 *
 * The algorithm:
 *   1. Back-project each 2D pixel (u, v) to a ray in the camera frame.
 *   2. Intersect the ray with the ground plane.
 *   3. Return the ordered 3D intersection points as the guide line.
 */
class GuideLineEstimator
{
public:
  explicit GuideLineEstimator(const ros::NodeHandle& pnh);

  /**
   * @brief Update the cached camera intrinsics.
   *
   * @param info sensor_msgs/CameraInfo containing K = [fx, 0, cx; 0, fy, cy; 0, 0, 1].
   */
  void setCameraInfo(const sensor_msgs::CameraInfoConstPtr& info);

  /**
   * @brief Check whether valid camera intrinsics have been received.
   */
  bool hasCameraInfo() const;

  /**
   * @brief Estimate the 3D guide line from the largest 2D contour.
   *
   * @param contour         2D contour points (pixel coordinates).
   * @param ground_normal   Ground plane normal (A, B, C).
   * @param ground_d        Ground plane offset D, where normal.dot(P) + D = 0.
   * @param sampled_contour Output sampled 2D points that produced valid 3D points.
   * @param guide_line      Output ordered 3D points on the ground plane.
   * @return true if at least one valid intersection was found.
   */
  bool estimate(const std::vector<cv::Point>& contour,
                const Eigen::Vector3f& ground_normal,
                float ground_d,
                std::vector<cv::Point>& sampled_contour,
                std::vector<Eigen::Vector3f>& guide_line) const;

  /**
   * @brief Fit a rectangle on the ground plane to a set of 3D points.
   *
   * The input points are assumed to lie near the ground plane. The result is a
   * rectangle (4 corners) centered at the input centroid and aligned with the
   * principal axes of the points projected onto the plane.
   *
   * @param points        3D points near/on the ground plane.
   * @param ground_normal Ground plane normal.
   * @param ground_d      Ground plane offset D.
   * @param rectangle     Output 4 rectangle corners in clockwise order.
   * @return true if a valid rectangle was fitted.
   */
  bool fitRectangleOnPlane(const std::vector<Eigen::Vector3f>& points,
                           const Eigen::Vector3f& ground_normal,
                           float ground_d,
                           std::vector<Eigen::Vector3f>& rectangle) const;

  /**
   * @brief Build a PointCloud2 message from 3D guide-line points.
   */
  sensor_msgs::PointCloud2 buildPointCloud2(const std_msgs::Header& header,
                                            const std::string& frame_id,
                                            const std::vector<Eigen::Vector3f>& points) const;

  /**
   * @brief Build a RViz Marker (LINE_STRIP) from 3D guide-line points.
   */
  visualization_msgs::Marker buildLineMarker(const std_msgs::Header& header,
                                             const std::string& frame_id,
                                             const std::vector<Eigen::Vector3f>& points) const;

  /**
   * @brief Build a Contour3DArray message from the original 2D contour and 3D points.
   */
  Contour3DArray buildContour3DArray(const std_msgs::Header& header,
                                     const std::vector<cv::Point>& contour_2d,
                                     const std::vector<Eigen::Vector3f>& points_3d,
                                     double area) const;

private:
  bool pixelToRay(int u, int v, Eigen::Vector3f& ray) const;
  bool rayPlaneIntersection(const Eigen::Vector3f& ray,
                            const Eigen::Vector3f& normal,
                            float d,
                            Eigen::Vector3f& point) const;

  sensor_msgs::CameraInfoConstPtr camera_info_;
  mutable std::mutex camera_info_mutex_;

  int max_contour_points_;
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_GUIDE_LINE_ESTIMATOR_H
