cmake_minimum_required(VERSION 3.0.2)
project(gemini_geometry_detector)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  geometry_msgs
  cv_bridge
  image_transport
  message_generation
  tf2
  tf2_ros
  ground_plane_calibrator
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  ContourInfo.msg
  ContourArray.msg
  GuideLineError.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    geometry_msgs
    cv_bridge
    image_transport
    message_runtime
    tf2
    tf2_ros
    ground_plane_calibrator
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(color_region_detector_node
  src/color_region_detector_node.cpp
  src/color_region_detector.cpp
  src/fit_line_guide_line_estimator.cpp
  src/depth_guide_line_estimator.cpp
  src/rectangle_guide_line_estimator.cpp
  src/topic_ground_plane_provider.cpp
  src/imu_ground_plane_provider.cpp
  src/imu_gravity_estimator.cpp
)

add_dependencies(color_region_detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(color_region_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS color_region_detector_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
