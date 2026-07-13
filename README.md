# gemini_geometry_detector

RGB-D 颜色区域检测与地面点云提取模块。

本包采用分阶段开发策略：
- **Phase 1**：基于 RGB 做颜色区域检测与轮廓提取（已完成）
- **Phase 2**：订阅 OrbbecSDK_ROS1 发布的对齐点云，进行地面分割（已完成）
- **Phase 3**（后续）：坐标系转换、地图匹配、到线距离计算

> **注意**：地面平面 A/B/C/D 的计算已迁移到独立的 `ground_plane_calibrator` 包。
> `gemini_geometry_detector` 现在通过订阅 `/ground_plane_calibrator/ground_plane/coefficients`
> 获取外部标定好的平面系数，不再每帧自行拟合。

---

## 1. 整体运行流程

```text
RGB-D 相机 / ROS bag
        |
        | /camera/depth_registered/points      (sensor_msgs/PointCloud2)
        v
+-----------------------------+
|  ground_plane_calibrator    |
|  自动采集 5~10s              |
|  计算地面平面 A/B/C/D        |
|  发布 /ground_plane/...     |
+-----------------------------+
        |
        | /ground_plane_calibrator/ground_plane/coefficients
        | (ground_plane_calibrator/PlaneCoefficients)
        v
+-----------------------------+
|  RGB-Cloud Sync             |  同步 RGB + PointCloud2
|  src/color_region_detector.cpp
+-----------------------------+
        |
        v
+-----------------------------+
|   ColorRegionDetector       |
| src/color_region_detector.cpp
|  ├─ BGR → HSV → inRange     |  颜色分割
|  ├─ morphologyEx            |  形态学
|  ├─ findContours            |  轮廓检测
|  ├─ buildContourInfo        |  2D 轮廓信息
|  └─ 外部 A/B/C/D 引导 3D     |  使用订阅的平面系数反投影轮廓
+-----------------------------+
        |
        | mask + annotated + ContourArray + depth_cloud
        v
+-----------------------------+
|           输出               |
+-----------------------------+
        |
        v
   /gemini_geometry_detector/color/mask
   /gemini_geometry_detector/color/annotated
   /gemini_geometry_detector/color/contours
   /gemini_geometry_detector/color/depth_cloud
```

### 1.1 输入阶段

源码：`src/color_region_detector.cpp` → `rgbCloudCallback()`

输入：
- `/camera/color/image_raw`：`sensor_msgs/Image`，BGR8 编码
- `/camera/depth_registered/points`：`sensor_msgs/PointCloud2`，已对齐到彩色图（`xyz` 或 `xyzrgb`）
- `/ground_plane_calibrator/ground_plane/coefficients`：
  `ground_plane_calibrator/PlaneCoefficients`，由 `ground_plane_calibrator` 发布的地面平面系数 A/B/C/D

处理流程：

1. **RGB-Cloud 同步**：使用 `message_filters::ApproximateTime` 同步彩色图与点云。
2. **cv_bridge 解码**：把 ROS 图像消息转成 `cv::Mat`。
3. **点云中心反查**：对检测到的 2D 轮廓中心 `(u, v)`，在有序点云中取 `index = v * width + u`，得到对应 3D 坐标并过滤无效轮廓。

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
3. **面积过滤**：剔除小于 `min_contour_area` 的轮廓。
4. **几何计算**：通过图像矩计算轮廓中心，通过 `boundingRect` 计算包围盒。
5. **下采样**：每个 2D 轮廓最多保留 `max_contour_points` 个点。

### 1.4 3D 引导线阶段

源码：`src/color_region_detector.cpp` → `processFrame()`

处理流程：

1. 接收外部标定节点发布的地面平面系数 A/B/C/D。
2. 对最大 2D 轮廓做矩形近似。
3. 利用相机内参将 2D 像素反投影为射线，与外部平面求交，得到 3D 引导线。
4. 转发原始深度点云到 `/color/depth_cloud`。

### 1.5 输出阶段

源码：`src/color_region_detector.cpp` → `publishResults()`

输出：

- `/gemini_geometry_detector/color/mask`：二值掩码（MONO8）
- `/gemini_geometry_detector/color/annotated`：带轮廓/中心/包围盒的标注图（BGR8）
- `/gemini_geometry_detector/color/contours`：2D 轮廓数组（`ContourArray.msg`）
- `/gemini_geometry_detector/color/depth_cloud`：完整深度 3D 点云（`sensor_msgs/PointCloud2`）
- `/gemini_geometry_detector/color/guide_line_cloud`：地面 3D 引导线点云（`sensor_msgs/PointCloud2`）
- `/gemini_geometry_detector/color/guide_line_marker`：地面 3D 引导线 RViz Marker（`visualization_msgs/Marker`，`LINE_STRIP`）
- `/gemini_geometry_detector/color/contours_3d`：最大轮廓的 3D 信息（`Contour3DArray.msg`）

---

## 2. 主要接口

### 2.1 ROS 接口

| 话题 | 类型 | 作用 |
|------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 输入彩色图像 |
| `/camera/depth_registered/points` | `sensor_msgs/PointCloud2` | Orbbec 输入对齐点云 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 彩色相机内参 |
| `/ground_plane_calibrator/ground_plane/coefficients` | `ground_plane_calibrator/PlaneCoefficients` | 外部地面平面 A/B/C/D |
| `/gemini_geometry_detector/color/mask` | `sensor_msgs/Image` | 二值掩码 |
| `/gemini_geometry_detector/color/annotated` | `sensor_msgs/Image` | 可视化标注图 |
| `/gemini_geometry_detector/color/contours` | `gemini_geometry_detector/ContourArray` | 2D 轮廓 |
| `/gemini_geometry_detector/color/depth_cloud` | `sensor_msgs/PointCloud2` | 完整深度 3D 点云 |
| `/gemini_geometry_detector/color/guide_line_cloud` | `sensor_msgs/PointCloud2` | 地面 3D 引导线点云 |
| `/gemini_geometry_detector/color/guide_line_marker` | `visualization_msgs/Marker` | 地面 3D 引导线可视化 |
| `/gemini_geometry_detector/color/contours_3d` | `gemini_geometry_detector/Contour3DArray` | 最大轮廓 3D 信息 |

自定义消息：

- `gemini_geometry_detector/ContourInfo.msg`
- `gemini_geometry_detector/ContourArray.msg`
- `gemini_geometry_detector/Contour3DInfo.msg`
- `gemini_geometry_detector/Contour3DArray.msg`
- `ground_plane_calibrator/PlaneCoefficients.msg`（外部包）

### 2.2 核心类

| 类 | 文件 | 职责 |
|---|---|---|
| `ColorRegionDetector` | `src/color_region_detector.cpp` | RGB-Cloud 同步、颜色分割、轮廓检测、结果发布 |
| `GuideLineEstimator` | `src/guide_line_estimator.cpp` | 2D 轮廓像素反投影、射线与地面求交、生成引导线消息 |
| `GroundPlaneCalibrator` | `../ground_plane_calibrator/src/ground_plane_calibrator.cpp` | 累积点云、地面分割、拟合地面平面、发布 A/B/C/D |
| `HybridGroundSegmenter` | `../ground_plane_calibrator/src/hybrid_ground_segmenter.cpp` | 地面点云分割 |
| `HybirdPreprocessor` | `../ground_plane_calibrator/src/hybird_preprocessor.cpp` | 地面分割前预处理 |

### 2.3 关键库接口

- `message_filters::Synchronizer` + `ApproximateTime`：RGB-Cloud 时间同步
- `cv_bridge::toCvShare`：ROS 图像 ↔ OpenCV
- `cv::inRange` / `cv::morphologyEx` / `cv::findContours`：图像处理
- `pcl::fromROSMsg` / `pcl::toROSMsg`：ROS 点云 ↔ PCL

---

## 3. 硬件环境

### 3.1 支持的传感器

本模块依赖 OrbbecSDK_ROS1 发布的对齐点云，因此适用于 Orbbec 系列 RGB-D 相机。

| 输入 | 话题示例 | 说明 |
|---|---|---|
| RGB 图像 | `/camera/color/image_raw` | `sensor_msgs/Image`，BGR8 |
| 对齐点云 | `/camera/depth_registered/points` | `sensor_msgs/PointCloud2` |

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
- `cv_bridge`、`image_transport`、`message_filters`
- `message_generation`、`message_runtime`
- `pcl_conversions`

系统依赖：

- OpenCV 3.2+（ROS Melodic 自带）
- PCL（ROS Melodic 自带）
- Eigen3

### 3.4 运行前提

- Orbbec 驱动已开启点云输出：`enable_point_cloud:=true`、`depth_registration:=true`、`ordered_pc:=true`。
- 点云和彩色图分辨率一致，且帧ID为彩色光心坐标系。

---

## 4. 参数与流程映射

| 参数 | 所在文件 | 作用 |
|------|---------|------|
| `input_topic` | `config/detector.yaml` | 输入彩色图像话题 |
| `point_cloud_topic` | `config/detector.yaml` | Orbbec 输入点云话题 |
| `camera_info_topic` | `config/detector.yaml` | 彩色相机内参话题 |
| `plane_coefficients_topic` | `config/detector.yaml` | 外部地面平面 A/B/C/D 话题 |
| `h_min` / `h_max` | `config/detector.yaml` | HSV 色调范围 |
| `s_min` / `s_max` | `config/detector.yaml` | HSV 饱和度范围 |
| `v_min` / `v_max` | `config/detector.yaml` | HSV 亮度范围 |
| `morph_kernel_size` | `config/detector.yaml` | 形态学核大小 |
| `min_contour_area` | `config/detector.yaml` | 轮廓最小面积阈值（原图分辨率） |
| `max_contour_points` | `config/detector.yaml` | 2D 轮廓最大点数 |
| `image_scale` | `config/detector.yaml` | 颜色检测和引导线的图像缩放比例 |
| `guide_line_every_n` | `config/detector.yaml` | 每隔 N 帧计算一次 3D 引导线 |

---

## 5. 面向开发者的重要约定

- **按职责分层**：
  - `ColorRegionDetector`：ROS 接口 + 2D 检测流程 + 点云转发
  - `GroundPlaneCalibrator`：ROS 接口 + 地面分割 + 平面拟合 + A/B/C/D 发布
  - `HybridGroundSegmenter`：纯 PCL 地面分割，不依赖 ROS
- **消息转换集中**：
  - 所有 ROS 消息构造在 `publishResults()` 中完成。
- **每帧 INFO 日志**：
  - 输出 2D 轮廓数量、过滤数量，便于调参。

---

## 6. 快速开始

### 6.1 编译

```bash
cd /home/jiancui1804/orbbec_ws
source /opt/ros/melodic/setup.bash
catkin_make
```

### 6.2 启动 Orbbec 相机

确保驱动开启对齐点云：

```bash
roslaunch orbbec_camera gemini_330_series.launch \
  enable_point_cloud:=true \
  depth_registration:=true \
  ordered_pc:=true \
  publish_tf:=true
```

### 6.3 启动地面平面标定节点

`gemini_geometry_detector` 现在依赖外部 A/B/C/D，必须先启动标定节点：

```bash
source /home/jiancui1804/orbbec_ws/devel/setup.bash
roslaunch ground_plane_calibrator calibrator.launch
```

### 6.4 启动检测节点

```bash
source /home/jiancui1804/orbbec_ws/devel/setup.bash
roslaunch gemini_geometry_detector color_detector.launch
```

### 6.5 启动检测节点 + RViz

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

# 查看标注图
rqt_image_view /gemini_geometry_detector/color/annotated
```

---

## 7. RViz 验证

`color_detector_rviz.launch` 会自动打开 RViz，显示：

1. **Grid**：地面网格。
2. **Annotated**：带绿色轮廓、红色中心点、蓝色包围盒的标注图。
3. **Mask**：HSV 阈值分割后的二值掩码。
4. **GuideLineMarker**：红色地面 3D 引导线（`LINE_STRIP`）。
5. **GuideLineCloud**：红色地面 3D 引导线点云。
6. **GroundPlaneMarker**（来自 `ground_plane_calibrator`）：绿色地面平面法向箭头。

固定帧为 `camera_color_optical_frame`。

---

## 8. 参数调参指南

### 8.1 HSV 调参

- 目标颜色偏色 → 调整 `h_min` / `h_max`
- 环境同色噪声 → 提高 `s_min` 和 `v_min`
- 目标发暗 → 降低 `v_min`

### 8.2 形态学与面积

- 掩码噪点多 → 增大 `morph_kernel_size`
- 误检小区域 → 增大 `min_contour_area`
- 远处目标漏检 → 减小 `min_contour_area`

### 8.3 点云相关

- 2D 轮廓中心在点云中查不到有效点 → 检查 Orbbec 驱动是否开启 `ordered_pc:=true`。
- 点云和彩色图分辨率不一致 → 检查驱动端 D2C / 深度配准是否开启。

---

## 9. 后续扩展（Phase 3）

- 订阅 `/tf`，把 `camera_color_optical_frame` 下的 3D 点转换到 `base_link` / `map`。
- 对 3D 轮廓点做 PCA/RANSAC 拟合 3D 直线或 2D 地面线。
- 与地图线匹配，计算车体到目标线的横向距离。

---

## 10. 关键源码索引

| 文件 | 作用 |
|---|---|
| `src/color_region_detector_node.cpp` | ROS 节点入口 |
| `include/gemini_geometry_detector/color_region_detector.h` | `ColorRegionDetector` 类声明 |
| `src/color_region_detector.cpp` | RGB-Cloud 同步、颜色分割、轮廓检测、发布 |
| `include/gemini_geometry_detector/guide_line_estimator.h` | `GuideLineEstimator` 类声明 |
| `src/guide_line_estimator.cpp` | 2D 像素反投影、射线-地面求交、消息生成 |
| `config/detector.yaml` | HSV / 形态学参数配置 |
| `launch/color_detector.launch` | 启动检测节点 |
| `launch/color_detector_rviz.launch` | 启动检测节点 + RViz |
| `../ground_plane_calibrator/launch/calibrator.launch` | 启动地面平面标定节点 |
| `rviz/color_detector.rviz` | RViz 配置文件 |
| `msg/ContourInfo.msg` / `ContourArray.msg` | 2D 轮廓消息 |
| `msg/Contour3DInfo.msg` / `Contour3DArray.msg` | 3D 轮廓消息 |
| `docs/color_region_detector.md` | 历史详细算法说明文档（已过期） |
