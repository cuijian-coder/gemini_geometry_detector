#ifndef GEMINI_GEOMETRY_DETECTOR_CAMERA_INTRINSICS_H
#define GEMINI_GEOMETRY_DETECTOR_CAMERA_INTRINSICS_H

namespace gemini_geometry_detector
{

/**
 * @brief Scaled camera intrinsics used for image-plane computations.
 */
struct CameraIntrinsics
{
  float fx = 0.0f;
  float fy = 0.0f;
  float cx = 0.0f;
  float cy = 0.0f;

  bool valid() const { return fx > 0.0f && fy > 0.0f; }
};

}  // namespace gemini_geometry_detector

#endif  // GEMINI_GEOMETRY_DETECTOR_CAMERA_INTRINSICS_H
