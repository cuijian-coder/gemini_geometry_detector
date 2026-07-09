# Contour information in pixel coordinates
int32 id
geometry_msgs/Point center        # (u, v, 0), contour centroid in pixel coords
float64 area                      # contour area in pixels^2
geometry_msgs/Point bbox_tl       # bounding box top-left  (u, v, 0)
geometry_msgs/Point bbox_br       # bounding box bottom-right (u, v, 0)
geometry_msgs/Point32[] points    # down-sampled contour points (optional)
