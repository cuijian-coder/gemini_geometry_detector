# gemini_geometry_detector

RGB 颜色区域检测与可插拔引导线误差估计模块。

本包支持两种引导线估计算法：

- **DepthGuideLineEstimator（默认）**：使用 RGB 图像、相机内参和地面平面，把 2D 轮廓反投影到地平面，在地平面坐标系中直接计算 `yaw_error`（rad）和 `lateral_error_m`（m）。
- **FitLineGuideLineEstimator**：仅依赖 RGB 图像和相机内参，在归一化图像平面拟合中心线，计算 `yaw_error`（rad）和 `lateral_error_n`（归一化图像平面横向偏移）。

引导线算法通过 `guide_line_estimator_type` 切换；Depth 模式所需的地面平面通过 `ground_plane_provider_type` 切换，支持话题订阅或 IMU 重力估计。

---

## 1. 整体运行流程

```text
RGB 相机 / ROS bag
        |
        | /camera/color/image_raw          (sensor_msgs/Image)
        | /camera/color/camera_info        (sensor_msgs/CameraInfo)
        | /tf                              (base_link → camera_color_optical_frame)
        | /camera/gyro_accel/sample        (sensor_msgs/Imu, IMU provider only)
        | /ground_plane/coefficients       (ground_plane_calibrator/PlaneCoefficients, topic provider only)
        v
+-----------------------------+
|   ColorRegionDetector       |
| src/color_region_detector.cpp
|  ├─ 加载 guide_line_estimator_type |  选择 FitLine / Depth 算法
|  ├─ 加载 ground_plane_provider_type|  选择 topic / imu 地面平面来源
|  ├─ TF 投影车辆前向轴              |  自动计算 target_angle（启动一次）
|  ├─ BGR → HSV → inRange            |  颜色分割
|  ├─ morphologyEx                   |  形态学
|  ├─ findContours                   |  轮廓检测
|  ├─ buildContourInfo               |  2D 轮廓信息
|  ├─ FitLine: cv::fitLine           |  归一化图像平面中心线拟合
|  └─ Depth: 反投影 + 地平面拟合     |  地平面 3D 引导线误差
+-----------------------------+
        |
        | mask + annotated + ContourArray + GuideLineError
        v
+-----------------------------+
|           输出               |
+-----------------------------+
```

### 1.1 输入阶段

源码：`src/color_region_detector.cpp` → `rgbCallback()`

输入：
- `/camera/color/image_raw`：`sensor_msgs/Image`，BGR8 编码
- `/camera/color/camera_info`：`sensor_msgs/CameraInfo`，彩色相机内参
- `/tf`：从 `base_frame` 到 `camera_frame` 的变换
- `/ground_plane/coefficients`：`ground_plane_calibrator/PlaneCoefficients`，当使用 `DepthGuideLineEstimator` 且 `ground_plane_provider_type=topic` 时必需
- `/camera/gyro_accel/sample`：`sensor_msgs/Imu`，当使用 `DepthGuideLineEstimator` 且 `ground_plane_provider_type=imu` 时必需

处理流程：

1. **加载估计算法**：根据 `guide_line_estimator_type` 创建 `FitLineGuideLineEstimator` 或 `DepthGuideLineEstimator`。
2. **加载地面平面来源（Depth 模式）**：根据 `ground_plane_provider_type` 创建 `TopicGroundPlaneProvider` 或 `ImuGroundPlaneProvider`，并通过回调把平面传给 `DepthGuideLineEstimator`。
3. **cv_bridge 解码**：把 ROS 图像消息转成 `cv::Mat`。
4. **CameraInfo 缓存**：收到内参后缓存原始内参（估计算法内部按需缩放，只处理第一次收到的消息）。
5. **TF 计算 target_angle（启动时一次）**：将 `base_link` 的 +X 轴（车辆前进方向）投影到相机图像平面，得到 `target_angle`。

### 1.2 颜色分割阶段

源码：`src/color_region_detector.cpp` → `createColorMask()`

处理流程：

1. **BGR → HSV**：把图像转换到 HSV 颜色空间。
2. **inRange 阈值分割**：按 YAML 中 `h_min/max`、`s_min/max`、`v_min/max` 提取目标颜色区域。

默认配置针对**黄色地面线**：

```yaml
h_min: 20
h_max: 40
s_min: 80
s_max: 255
v_min: 80
v_max: 255
```

### 1.3 形态学与轮廓阶段

源码：`src/color_region_detector.cpp` → `applyMorphology()` / `detectContours()` / `buildContourInfo()`

处理流程：

1. **形态学开/闭运算**：去除小噪点并填补目标区域内部空洞。
2. **findContours**：只检测最外层轮廓（`RETR_EXTERNAL`）。
3. **长度过滤**：剔除周长小于 `min_contour_length` 的轮廓（使用周长而非面积，避免细长水平线被误过滤）。
4. **几何计算**：通过图像矩计算轮廓中心，通过 `boundingRect` 计算包围盒。
5. **下采样**：每个 2D 轮廓最多保留 `max_contour_points` 个点。

### 1.4 引导线拟合阶段

源码：`src/color_region_detector.cpp` → `processFrame()`

两种实现都遵循相同的流程框架：

1. 启动时通过 TF 自动计算 `target_angle`（只计算一次，之后固定使用）。
2. 在缩放后的图像中选取面积最大的轮廓 `largest_contour`。
3. 用 `cv::minAreaRect` 计算最小外接矩形，**仅用于可视化**。
4. 调用选中的 `IGuideLineEstimator::estimate()`。

#### FitLineGuideLineEstimator

- 把 `largest_contour` 的像素点归一化到图像平面：
  ```text
  x_n = (u - cx) / fx
  y_n = (v - cy) / fy
  ```
- 在归一化坐标系中调用 `cv::fitLine()` 拟合中心线。
- 在 ROI 行（默认图像高度 75% 处）求中心线 x 坐标：
  ```text
  lateral_error_n = x_roi_n
  ```
- 计算方向角误差：
  ```text
  yaw_error = smallest_angle(line_direction, target_angle)
  ```
  由于 `cv::fitLine` 可能返回同一直线的任意一个方向，取两个相反方向中误差绝对值较小的那个。

#### DepthGuideLineEstimator

- 把 `largest_contour` 的像素点反投影为相机坐标系射线，并与地面平面求交，得到地平面 3D 点云。
- 在地平面上对 3D 点做 PCA，拟合地平面直线。
- 将直线方向对齐到车辆前进方向，计算：
  ```text
  yaw_error      = angle(line_direction, vehicle_forward_on_ground) - target_angle
  lateral_error_m = signed_distance(look_ahead_reference_point, ground_line)
  ```
- `look_ahead_m` 控制参考点在车体前方多远（默认为 `0`，即相机光心在地平面的投影点）。

### 1.5 输出阶段

源码：`src/color_region_detector.cpp` → `publishResults()`

输出：

- `/gemini_geometry_detector/color/mask`：二值掩码（MONO8）
- `/gemini_geometry_detector/color/annotated`：带轮廓、中心线、ROI、误差的标注图（BGR8）
- `/gemini_geometry_detector/color/contours`：2D 轮廓数组（`ContourArray.msg`）
- `/gemini_geometry_detector/color/guide_line_error`：引导线误差（`GuideLineError.msg`）

---

## 2. 主要接口

### 2.1 ROS 接口

| 话题 | 类型 | 作用 |
|------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 输入彩色图像 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 彩色相机内参 |
| `/tf` | `tf2_msgs/TFMessage` | 车辆坐标系到相机坐标系的变换 |
| `/ground_plane/coefficients` | `ground_plane_calibrator/PlaneCoefficients` | 地面平面系数（topic provider 模式） |
| `/camera/gyro_accel/sample` | `sensor_msgs/Imu` | IMU 数据（imu provider 模式） |
| `/gemini_geometry_detector/color/mask` | `sensor_msgs/Image` | 二值掩码 |
| `/gemini_geometry_detector/color/annotated` | `sensor_msgs/Image` | 可视化标注图 |
| `/gemini_geometry_detector/color/contours` | `gemini_geometry_detector/ContourArray` | 2D 轮廓 |
| `/gemini_geometry_detector/color/guide_line_error` | `gemini_geometry_detector/GuideLineError` | 角度 + 横向误差（n/m） |

自定义消息：

- `gemini_geometry_detector/ContourInfo.msg`
- `gemini_geometry_detector/ContourArray.msg`
- `gemini_geometry_detector/GuideLineError.msg`

### 2.2 核心类

| 类 | 文件 | 职责 |
|---|---|---|
| `ColorRegionDetector` | `src/color_region_detector.cpp` | RGB 订阅、颜色分割、轮廓检测、算法调度、误差发布 |
| `IGuideLineEstimator` | `include/.../guide_line_estimator_interface.h` | 引导线估计算法接口 |
| `FitLineGuideLineEstimator` | `src/fit_line_guide_line_estimator.cpp` | RGB 图像平面引导线拟合 |
| `DepthGuideLineEstimator` | `src/depth_guide_line_estimator.cpp` | 地平面 3D 引导线拟合 |
| `IGroundPlaneProvider` | `include/.../ground_plane_provider_interface.h` | 地面平面来源接口 |
| `TopicGroundPlaneProvider` | `src/topic_ground_plane_provider.cpp` | 订阅话题获取地面平面 |
| `ImuGroundPlaneProvider` | `src/imu_ground_plane_provider.cpp` | 从 IMU 重力估计地面平面 |

### 2.3 关键库接口

- `cv_bridge::toCvShare`：ROS 图像 ↔ OpenCV
- `cv::inRange` / `cv::morphologyEx` / `cv::findContours`：图像处理
- `cv::fitLine`：归一化图像平面中心线拟合

---

## 3. 硬件环境

### 3.1 支持的传感器

任何能发布 RGB 图像和对应 `CameraInfo` 的相机均可使用。

| 输入 | 话题示例 | 说明 |
|---|---|---|
| RGB 图像 | `/camera/color/image_raw` | `sensor_msgs/Image`，BGR8 |
| 相机内参 | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` |

### 3.2 所需主机

- 操作系统：Ubuntu 18.04（ROS Melodic）
- 构建工具：`catkin_make`
- C++ 标准：C++14
- CPU：x86_64 台式机/工控机；纯 CPU 实现
- GPU：**不需要**

### 3.3 所需依赖

`package.xml` 中已声明：

- `roscpp`
- `sensor_msgs`、`std_msgs`、`geometry_msgs`
- `cv_bridge`、`image_transport`
- `message_generation`、`message_runtime`
- `ground_plane_calibrator`（Depth 模式必需）

系统依赖：

- OpenCV 3.2+（ROS Melodic 自带）
- Eigen3

---

## 4. 参数与流程映射

| 参数 | 所在文件 | 作用 |
|------|---------|------|
| `input_topic` | `config/detector.yaml` | 输入彩色图像话题 |
| `camera_info_topic` | `config/detector.yaml` | 彩色相机内参话题 |
| `h_min` / `h_max` | `config/detector.yaml` | HSV 色调范围 |
| `s_min` / `s_max` | `config/detector.yaml` | HSV 饱和度范围 |
| `v_min` / `v_max` | `config/detector.yaml` | HSV 亮度范围 |
| `morph_kernel_size` | `config/detector.yaml` | 形态学核大小 |
| `min_contour_length` | `config/detector.yaml` | 轮廓最小周长阈值（原图分辨率，使用 `cv::arcLength`） |
| `max_contour_points` | `config/detector.yaml` | 轮廓最大点数（拟合 + 发布） |
| `min_aspect_ratio` | `config/detector.yaml` | 轮廓最小长宽比（长边/短边），用于过滤非线状区域 |
| `max_aspect_ratio` | `config/detector.yaml` | 轮廓最大长宽比，`<=0` 表示不限制 |
| `image_scale` | `config/detector.yaml` | 颜色检测和引导线的图像缩放比例 |
| `use_tf_target_angle` | `config/detector.yaml` | 是否通过 TF 自动计算 `target_angle` |
| `target_angle` | `config/detector.yaml` | 目标线方向（rad），TF 失败时作为回退；FitLine 为图像平面角，Depth 为地平面相对车体前向的角 |
| `base_frame` | `config/detector.yaml` | 车辆坐标系，默认 `base_link` |
| `camera_frame` | `config/detector.yaml` | 相机光心坐标系，默认 `camera_color_optical_frame` |
| `roi_y_ratio` | `config/detector.yaml` | ROI 行相对图像高度的比例 |
| `roi_y` | `config/detector.yaml` | ROI 行绝对像素值，`-1` 表示使用 `roi_y_ratio` |
| `guide_line_estimator_type` | `config/detector.yaml` | 引导线估计算法：`DepthGuideLineEstimator` / `FitLineGuideLineEstimator` |
| `look_ahead_m` | `config/detector.yaml` | Depth 模式下前视参考点距离（m），默认 `0` |
| `ground_plane_provider_type` | `config/detector.yaml` | 地面平面来源：`topic` / `imu` |
| `ground_plane_topic` | `config/detector.yaml` | topic provider 订阅的话题 |
| `imu_topic` | `config/detector.yaml` | imu provider 订阅的 IMU 话题 |
| `camera_height` | `config/detector.yaml` | imu provider 的相机高度（m） |
| `gravity_filter_alpha` | `config/detector.yaml` | imu provider 重力低通滤波系数 |
| `imu_to_camera_qx/qy/qz/qw` | `config/detector.yaml` | imu provider 的 IMU 到相机旋转四元数 |

---

## 5. 面向开发者的重要约定

- **按职责分层**：
  - `ColorRegionDetector`：ROS 接口 + 2D 检测流程 + 算法调度
  - `IGuideLineEstimator`：可插拔的引导线误差计算接口
  - `IGroundPlaneProvider`：可插拔的地面平面来源接口
- **归一化坐标（FitLine 模式）**：
  - 图像平面拟合和误差计算在归一化图像平面 `(x_n, y_n)` 完成，结果对相机分辨率/焦距更鲁棒。
- **地平面坐标（Depth 模式）**：
  - 反投影到相机坐标系并与地面平面求交，在地平面直接计算米制横向误差 `lateral_error_m`。
- **每帧 INFO 日志**：
  - 输出 2D 轮廓数量、过滤数量、引导线是否有效，便于调参。

---

## 6. 快速开始

### 6.1 编译

```bash
cd /home/jiancui1804/orbbec_ws
source /opt/ros/melodic/setup.bash
catkin_make
```

### 6.2 启动相机

确保相机驱动发布 RGB 图像和 `CameraInfo`：

```bash
roslaunch orbbec_camera gemini_330_series.launch
```

### 6.3 启动检测节点

```bash
source /home/jiancui1804/orbbec_ws/devel/setup.bash
roslaunch gemini_geometry_detector color_detector.launch
```

### 6.4 启动检测节点 + RViz

```bash
source /home/jiancui1804/orbbec_ws/devel/setup.bash
roslaunch gemini_geometry_detector color_detector_rviz.launch
```

### 6.5 配合 bag 使用

```bash
# 终端 1：启动检测 + RViz
roslaunch gemini_geometry_detector color_detector_rviz.launch

# 终端 2：播放 bag
rosbag play your_bag.bag -l
```

### 6.6 查看输出

```bash
# 查看 2D 轮廓
rostopic echo /gemini_geometry_detector/color/contours

# 查看引导线误差
rostopic echo /gemini_geometry_detector/color/guide_line_error

# 查看标注图
rqt_image_view /gemini_geometry_detector/color/annotated
```

---

## 7. RViz 验证

`color_detector_rviz.launch` 会自动打开 RViz，显示：

1. **Grid**：地面网格。
2. **Annotated**：带绿色轮廓、红色中心点、蓝色包围盒、黄色 ROI 线、绿色中心线、品红色横向误差线段、`yaw` / `lat` 文字的标注图。
3. **Mask**：HSV 阈值分割后的二值掩码。

固定帧为 `camera_color_optical_frame`。

---

## 8. 参数调参指南

### 8.1 HSV 调参

- 目标颜色偏色 → 调整 `h_min` / `h_max`
- 环境同色噪声 → 提高 `s_min` 和 `v_min`
- 目标发暗 → 降低 `v_min`

### 8.2 形态学与长度

- 掩码噪点多 → 增大 `morph_kernel_size`
- 误检短线段 → 增大 `min_contour_length`
- 远处目标漏检 → 减小 `min_contour_length`
- 细长水平线被过滤 → 已改用 `cv::arcLength`，按实际长度调整即可

### 8.3 引导线方向

- 车辆直行时引导线在图像中向上 → 保持 `target_angle = -pi/2`。
- 如果安装角度导致目标方向不同，按实际图像方向调整。

### 8.4 ROI 行

- 默认 `roi_y_ratio = 0.75`（图像下方 1/4 处），越靠近车辆横向误差越敏感。
- 如需固定像素行，设置 `roi_y` 为具体值（例如 `roi_y: 600`），`roi_y_ratio` 会被忽略。

---

## 9. 后续扩展

- 订阅 `/tf`，把图像平面误差转换到车体坐标系。
- 下游控制器使用 `GuideLineError`：
  ```text
  angular_z = -Kp_yaw * yaw_error - Kp_pixel * lateral_error_n
  ```
- 与地图线匹配，计算车体到目标线的横向距离。

---

## 10. 关键源码索引

| 文件 | 作用 |
|---|---|
| `src/color_region_detector_node.cpp` | ROS 节点入口 |
| `include/gemini_geometry_detector/color_region_detector.h` | `ColorRegionDetector` 类声明 |
| `src/color_region_detector.cpp` | RGB 处理、轮廓检测、算法调度、发布 |
| `src/fit_line_guide_line_estimator.cpp` | RGB 图像平面引导线估计算法 |
| `src/depth_guide_line_estimator.cpp` | 地平面 3D 引导线估计算法 |
| `include/gemini_geometry_detector/guide_line_estimator_interface.h` | 引导线估计算法接口 |
| `include/gemini_geometry_detector/ground_plane_provider_interface.h` | 地面平面来源接口 |
| `src/topic_ground_plane_provider.cpp` | 话题地面平面来源实现 |
| `src/imu_ground_plane_provider.cpp` | IMU 地面平面来源实现 |
| `config/detector.yaml` | HSV / 形态学 / 引导线参数配置 |
| `launch/color_detector.launch` | 启动检测节点 |
| `launch/color_detector_rviz.launch` | 启动检测节点 + RViz |
| `rviz/color_detector.rviz` | RViz 配置文件 |
| `msg/ContourInfo.msg` / `ContourArray.msg` | 2D 轮廓消息 |
| `msg/GuideLineError.msg` | 引导线误差消息 |
| `docs/color_region_detector.md` | 上游轮廓检测、过滤与合并说明 |
| `docs/depth_guide_line_estimator.md` | DepthGuideLineEstimator 反投影与误差计算说明 |
