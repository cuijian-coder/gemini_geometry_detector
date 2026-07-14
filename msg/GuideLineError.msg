# Guide-line control errors.
# Depending on the active estimator, either lateral_error_n or lateral_error_m
# will be populated:
#   - RGB-only estimator (FitLineGuideLineEstimator): lateral_error_n
#   - Depth/ground-plane estimator (DepthGuideLineEstimator): lateral_error_m
std_msgs/Header header
float64 yaw_error          # rad, line direction minus target direction
float64 lateral_error_n    # normalized image-plane lateral error = (x_roi - cx) / fx
float64 lateral_error_m    # ground-plane lateral error in meters (depth estimator only)
bool valid                 # true if a valid guide line was detected
geometry_msgs/Point roi_point  # (u, v, 0) of the evaluated center-line point in original image coords
