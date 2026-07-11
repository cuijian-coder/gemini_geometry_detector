# 3D contour information in camera frame (meters)
int32 id
geometry_msgs/Point center_2d       # (u, v, 0) contour centroid in pixel coords
geometry_msgs/Point center_3d       # (X, Y, Z) contour centroid in camera frame, meters
float64 area                        # contour area in pixels^2
geometry_msgs/Point bbox_tl         # bounding box top-left (u, v, 0)
geometry_msgs/Point bbox_br         # bounding box bottom-right (u, v, 0)
geometry_msgs/Point32[] points_2d   # down-sampled 2D contour points
geometry_msgs/Point[] points_3d     # corresponding 3D contour points (optional)
float64 mean_depth                  # mean depth of sampled contour points, meters
float64 min_depth                   # minimum depth of sampled contour points, meters
float64 max_depth                   # maximum depth of sampled contour points, meters
