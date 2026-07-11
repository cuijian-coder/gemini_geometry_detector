# gemini_geometry_detector

RGB-D 颜色区域检测与 3D 轮廓提取模块。

本包采用分阶段开发策略：
- **Phase 1**：基于 RGB 做颜色区域检测与轮廓提取（已完成）
- **Phase 2**：加入深度图和相机内参，把 2D 轮廓投影到相机坐标系 3D 点（已完成）
- **Phase 3**（后续）：坐标系转换、地图匹配、到线距离计算

---

## 1. 整体运行流程

```text
RGB-D 相机 / ROS bag
        |
        | /camera/color/image_raw       (sensor_msgs/Image)
        | /camera/depth/image_raw       (sensor_msgs/Image)
        | /camera/color/camera_info     (sensor_msgs/CameraInfo)
        v
+---------------------------+
|  RGB-D Sync (ApproximateTime)  |  同步 RGB + Depth + CameraInfo
|  src/color_region_detector.cpp
+---------------------------+
        |
        v
+---------------------------+
|   ColorRegionDetector     |
| src/color_region_detector.cpp
|  ├─ BGR → HSV → inRange   |  颜色分割
|  ├─ morphologyEx          |  形态学
|  ├─ findContours          |  轮廓检测
|  ├─ buildContourInfo      |  2D 轮廓信息
|  └─ DepthProjector        |  2D → 3D 投影
|     src/depth_projector.cpp
+---------------------------+
        |
        | mask + annotated + ContourArray + Contour3DArray
        v
+---------------------------+
|         输出               |
+---------------------------+
        |
        v
   /gemini_geometry_detector/color/mask
   /gemini_geometry_detector/color/annotated
   /gemini_geometry_detector/color/contours
   /gemini_geometry_detector/color/contours_3d
```

### 1.1 输入阶段

源码：`src/color_region_detector.cpp` → `rgbDepthInfoCallback()`

输入：
- `/camera/color/image_raw`：`sensor_msgs/Image`，BGR8 编码
- `/camera/depth/image_raw`：`sensor_msgs/Image`，16UC1（毫米）或 32FC1（米）
- `/camera/color/camera_info`：`sensor_msgs/CameraInfo`，提供 `fx, fy, cx, cy`

处理流程：

1. **RGB-D 同步**：使用 `message_filters::ApproximateTime` 同步三者的回调。
2. **cv_bridge 解码**：把 ROS 图像消息转成 `cv::Mat`。
3. **分辨率检查**：如果 RGB 和 Depth 分辨率不一致，打印 WARN。

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

### 1.4 3D 投影阶段

源码：`src/depth_projector.cpp` → `DepthProjector::projectContour()`

处理流程：

1. 从 `CameraInfo.K` 提取相机内参 `fx, fy, cx, cy`。
2. 对 2D 轮廓中心 `(u, v)` 查询深度 `Z`。
3. 用公式计算相机坐标系下的 3D 坐标：

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth(u, v)
```

4. 对轮廓采样点做同样的投影，计算 `mean_depth` / `min_depth` / `max_depth`。
5. 过滤深度不在 `[min_depth_m, max_depth_m]` 范围内的轮廓。

### 1.5 输出阶段

源码：`src/color_region_detector.cpp` → `publishResults()`

输出：

- `/gemini_geometry_detector/color/mask`：二值掩码（MONO8）
- `/gemini_geometry_detector/color/annotated`：带轮廓/中心/包围盒的标注图（BGR8）
- `/gemini_geometry_detector/color/contours`：2D 轮廓数组（`ContourArray.msg`）
- `/gemini_geometry_detector/color/contours_3d`：3D 轮廓数组（`Contour3DArray.msg`，新增）
- `/gemini_geometry_detector/color/ground_cloud`：地面点云（`sensor_msgs/PointCloud2`）
- `/gemini_geometry_detector/color/depth_cloud`：完整深度 3D 点云（`sensor_msgs/PointCloud2`）

---

## 2. 主要接口

### 2.1 ROS 接口

| 话题 | 类型 | 作用 |
|------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 输入彩色图像 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 输入深度图像 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 彩色相机内参 |
| `/gemini_geometry_detector/color/mask` | `sensor_msgs/Image` | 二值掩码 |
| `/gemini_geometry_detector/color/annotated` | `sensor_msgs/Image` | 可视化标注图 |
| `/gemini_geometry_detector/color/contours` | `gemini_geometry_detector/ContourArray` | 2D 轮廓 |
| `/gemini_geometry_detector/color/contours_3d` | `gemini_geometry_detector/Contour3DArray` | 3D 轮廓 |
| `/gemini_geometry_detector/color/ground_cloud` | `sensor_msgs/PointCloud2` | 地面点云 |
| `/gemini_geometry_detector/color/depth_cloud` | `sensor_msgs/PointCloud2` | 完整深度 3D 点云 |

自定义消息：

- `gemini_geometry_detector/ContourInfo.msg`
- `gemini_geometry_detector/ContourArray.msg`
- `gemini_geometry_detector/Contour3DInfo.msg`
- `gemini_geometry_detector/Contour3DArray.msg`

`Contour3DInfo` 字段：

| 字段 | 说明 |
|------|------|
| `id` | 轮廓编号 |
| `center_2d` | 轮廓中心像素坐标 `(u, v, 0)` |
| `center_3d` | 轮廓中心相机坐标 `(X, Y, Z)`，单位：米 |
| `area` | 轮廓面积（像素²） |
| `bbox_tl` / `bbox_br` | 2D 包围盒 |
| `points_2d` | 下采样 2D 轮廓点 |
| `points_3d` | 对应 3D 轮廓点（可选） |
| `mean_depth` / `min_depth` / `max_depth` | 采样点深度统计，单位：米 |

### 2.2 核心类

| 类 | 文件 | 职责 |
|---|---|---|
| `ColorRegionDetector` | `src/color_region_detector.cpp` | RGB-D 同步、颜色分割、轮廓检测、结果发布 |
| `DepthProjector` | `src/depth_projector.cpp` | 深度图解析、相机模型、2D→3D 投影 |

### 2.3 关键库接口

- `message_filters::Synchronizer` + `ApproximateTime`：RGB-D 时间同步
- `cv_bridge::toCvShare`：ROS 图像 ↔ OpenCV
- `cv::inRange` / `cv::morphologyEx` / `cv::findContours`：图像处理
- `sensor_msgs::CameraInfo::K`：相机内参

---

## 3. 硬件环境

### 3.1 支持的传感器

本模块对具体硬件依赖很小，只要是发布标准 ROS 消息的 RGB-D 相机或 RGB 相机都可以接入。

| 输入 | 话题示例 | 说明 |
|---|---|---|
| RGB 图像 | `/camera/color/image_raw` | `sensor_msgs/Image`，BGR8 |
| 深度图像 | `/camera/depth/image_raw` | `sensor_msgs/Image`，16UC1 或 32FC1 |
| 相机内参 | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` |

> 如果只有 RGB 没有 Depth，节点会因为没有同步数据而无法处理。Phase 1 的纯 RGB 版本可通过切换配置文件恢复。

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

系统依赖：

- OpenCV 3.2+（ROS Melodic 自带）
- Eigen3（后续 Phase 使用）

### 3.4 运行前提

- 输入彩色图像、深度图像、相机内参话题都已发布。
- 深度图已与彩色图对齐（D2C），或至少分辨率一致。

---

## 4. 参数与流程映射

| 参数 | 所在文件 | 作用 |
|------|---------|------|
| `input_topic` | `config/detector.yaml` | 输入彩色图像话题 |
| `depth_topic` | `config/detector.yaml` | 输入深度图像话题 |
| `camera_info_topic` | `config/detector.yaml` | 相机内参话题 |
| `point_cloud_topic` | `config/detector.yaml` | Orbbec 输入点云话题 |
| `use_point_cloud` | `config/detector.yaml` | 是否使用 Orbbec 点云代替深度图 |
| `h_min` / `h_max` | `config/detector.yaml` | HSV 色调范围 |
| `s_min` / `s_max` | `config/detector.yaml` | HSV 饱和度范围 |
| `v_min` / `v_max` | `config/detector.yaml` | HSV 亮度范围 |
| `morph_kernel_size` | `config/detector.yaml` | 形态学核大小 |
| `min_contour_area` | `config/detector.yaml` | 轮廓最小面积阈值 |
| `max_contour_points` | `config/detector.yaml` | 2D 轮廓最大点数 |
| `sample_3d_step` | `config/detector.yaml` | 3D 投影采样步长 |
| `publish_points_3d` | `config/detector.yaml` | 是否发布 3D 轮廓点 |
| `depth_scale` | `config/detector.yaml` | 深度 raw value 转米（16UC1 用 0.001） |
| `min_depth_m` / `max_depth_m` | `config/detector.yaml` | 有效深度范围 |

---

## 5. 面向开发者的重要约定

- **按职责分层**：
  - `ColorRegionDetector`：ROS 接口 + 2D 检测流程
  - `DepthProjector`：纯几何投影，不依赖 ROS
- **算法与 ROS 解耦**：
  - `DepthProjector` 只接收 `cv::Mat` 和 `CameraInfo`，便于单元测试。
- **消息转换集中**：
  - 所有 ROS 消息构造在 `publishResults()` 中完成。
- **每帧 INFO 日志**：
  - 输出 2D/3D 轮廓数量、过滤数量，便于调参。

---

## 6. 快速开始

### 6.1 编译

```bash
cd /home/jiancui1804/orbbec_ws
source /opt/ros/melodic/setup.bash
catkin_make
```

### 6.2 启动检测节点

确保输入话题已经发布，然后：

```bash
source /home/jiancui1804/orbbec_ws/devel/setup.bash
roslaunch gemini_geometry_detector color_detector.launch
```

### 6.3 启动检测节点 + RViz

```bash
source /home/jiancui1804/orbbec_ws/devel/setup.bash
roslaunch gemini_geometry_detector color_detector_rviz.launch
```

### 6.4 使用 OrbbecSDK_ROS1 点云输入

在启动 Orbbec 驱动时开启 `enable_point_cloud`、`depth_registration`、`ordered_pc`，然后：

```bash
source /home/jiancui1804/orbbec_ws/devel/setup.bash
roslaunch gemini_geometry_detector color_detector_rviz.launch use_point_cloud:=true
```

### 6.5 同时对比 DepthProjector 与 Orbbec 点云

启动两个检测节点实例，一个走 `DepthProjector`（深度图模式），一个走 Orbbec 点云，RViz 里同时显示两份完整点云和地面点云：

```bash
source /home/jiancui1804/orbbec_ws/devel/setup.bash
roslaunch gemini_geometry_detector compare_point_clouds.launch
```

RViz 中：
- **白色**：`DepthProjector` 生成的完整点云
- **红色**：OrbbecSDK_ROS1 发布的完整点云
- **绿色**：`DepthProjector` 模式分割出的地面点云
- **黄色**：Orbbec 点云模式分割出的地面点云

> 固定帧为 `camera_link`，需要相机驱动发布对应的 TF。

### 6.6 配合 bag 使用

```bash
# 终端 1：启动检测 + RViz
roslaunch gemini_geometry_detector color_detector_rviz.launch

# 终端 2：播放 bag
rosbag play your_bag.bag -l
```

### 6.7 查看输出

```bash
# 查看 3D 轮廓数值
rostopic echo /gemini_geometry_detector/color/contours_3d

# 查看标注图
rqt_image_view /gemini_geometry_detector/color/annotated
```

---

## 7. RViz 验证

`color_detector_rviz.launch` 会自动打开 RViz，显示：

1. **Annotated**：带绿色轮廓、红色中心点、蓝色包围盒的标注图
2. **Mask**：HSV 阈值分割后的二值掩码

`Contour3DArray` 目前通过 `rostopic echo` 查看数值，Phase 3 会加入 Marker 可视化。

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

### 8.3 深度过滤

- 太近/太远的轮廓被过滤 → 调整 `min_depth_m` / `max_depth_m`
- 深度抖动大 → 增大 `sample_3d_step` 或关闭 `publish_points_3d`

### 8.4 3D 投影

- 如果深度编码是 `32FC1`（米），把 `depth_scale` 改为 `1.0`。
- 如果深度未与彩色对齐，需先确保驱动端 D2C 开启。

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
| `src/color_region_detector.cpp` | RGB-D 同步、颜色分割、轮廓检测、发布 |
| `include/gemini_geometry_detector/depth_projector.h` | `DepthProjector` 类声明 |
| `src/depth_projector.cpp` | 深度解析、2D→3D 投影 |
| `config/detector.yaml` | HSV / 形态学 / 深度 / 3D 参数配置 |
| `launch/color_detector.launch` | 启动检测节点 |
| `launch/color_detector_rviz.launch` | 启动检测节点 + RViz |
| `rviz/color_detector.rviz` | RViz 配置文件 |
| `msg/ContourInfo.msg` / `ContourArray.msg` | 2D 轮廓消息 |
| `msg/Contour3DInfo.msg` / `Contour3DArray.msg` | 3D 轮廓消息 |
| `docs/color_region_detector.md` | 详细算法说明文档 |
