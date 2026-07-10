# gemini_geometry_detector

RGB 颜色区域检测与轮廓提取模块。

本包采用分阶段开发策略：**Phase 1 先基于 RGB 做颜色区域检测与轮廓提取**；后续 Phase 2/3 再逐步加入深度验证、相机坐标转换、地图匹配与距离计算。

---

## 1. 整体运行流程

```text
RGB 相机 / ROS bag
        |
        | /camera/color/image_raw  (sensor_msgs/Image)
        v
+---------------------------+
|    ImageSubscriber        |  cv_bridge 解码为 cv::Mat (BGR)
|  src/color_region_detector.cpp
+---------------------------+
        |
        v
+---------------------------+
|   ColorRegionDetector     |  HSV 阈值分割 → 形态学 → 轮廓检测
| src/color_region_detector.cpp
+---------------------------+
        |
        | ContourArray + mask + annotated image
        v
+---------------------------+
|         输出               |
+---------------------------+
        |
        v
   /gemini_geometry_detector/color/mask
   /gemini_geometry_detector/color/annotated
   /gemini_geometry_detector/color/contours
```

### 1.1 输入阶段

源码：`src/color_region_detector.cpp` → `imageCallback()` / `getLatestImage()` / `convertToCvImage()`

输入：`/camera/color/image_raw`（`sensor_msgs/Image`，BGR8 编码）

处理流程：

1. **最新帧缓冲**：图像回调仅做线程安全地缓存，不阻塞高频图像流。
2. **定时取帧**：以固定频率（默认 10Hz）取走最新帧，降低 CPU 占用。
3. **cv_bridge 解码**：使用 `cv_bridge::toCvShare` 零拷贝转成 `cv::Mat`。

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
5. **下采样**：每个轮廓最多保留 `max_contour_points` 个点，减少消息体积。

### 1.4 输出阶段

源码：`src/color_region_detector.cpp` → `publishResults()`

输出：

- `/gemini_geometry_detector/color/mask`：二值掩码（MONO8）
- `/gemini_geometry_detector/color/annotated`：带轮廓/中心/包围盒的标注图（BGR8）
- `/gemini_geometry_detector/color/contours`：轮廓数组（自定义 `ContourArray.msg`）

---

## 2. 主要接口

### 2.1 ROS 接口

| 话题 | 类型 | 作用 |
|------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 输入彩色图像（由相机驱动或 bag 发布） |
| `/gemini_geometry_detector/color/mask` | `sensor_msgs/Image` | 颜色分割后的二值掩码 |
| `/gemini_geometry_detector/color/annotated` | `sensor_msgs/Image` | 可视化标注图 |
| `/gemini_geometry_detector/color/contours` | `gemini_geometry_detector/ContourArray` | 轮廓数值输出 |

自定义消息：

- `gemini_geometry_detector/ContourInfo.msg`
- `gemini_geometry_detector/ContourArray.msg`

`ContourInfo` 字段：

| 字段 | 说明 |
|------|------|
| `id` | 轮廓编号 |
| `center` | 轮廓中心像素坐标 `(u, v, 0)` |
| `area` | 轮廓面积（像素²） |
| `bbox_tl` | 包围盒左上角 `(u, v, 0)` |
| `bbox_br` | 包围盒右下角 `(u, v, 0)` |
| `points` | 下采样后的轮廓点序列 |

### 2.2 OpenCV 接口

- `cv::cvtColor`：BGR → HSV
- `cv::inRange`：HSV 阈值分割
- `cv::getStructuringElement` / `cv::morphologyEx`：形态学去噪
- `cv::findContours`：轮廓提取
- `cv::moments` / `cv::contourArea` / `cv::boundingRect`：轮廓几何计算

### 2.3 ROS API

- `ros::init`、`ros::NodeHandle`、`ros::Timer`
- `image_transport::Subscriber` / `Publisher`
- `cv_bridge::toCvShare`
- `sensor_msgs/Image`

---

## 3. 硬件环境

### 3.1 支持的相机

本模块对具体硬件依赖很小，只要是发布标准 `sensor_msgs/Image` 的 RGB 相机都可以接入。

常见接入方式：

| 来源 | 话题示例 | 说明 |
|------|---------|------|
| USB 相机驱动 | `/camera/color/image_raw` | 通用 ROS 相机驱动 |
| RGB-D 相机驱动 | `/camera/color/image_raw` | 如 Orbbec、RealSense 等 |
| ROS bag | `/camera/color/image_raw` | 录制好的彩色图像 bag |

### 3.2 所需主机

- 操作系统：Ubuntu 18.04（ROS Melodic）
- 构建工具：`catkin_make`
- C++ 标准：C++14
- CPU：x86_64 台式机/工控机；当前实现为纯 CPU
- GPU：**不需要**

### 3.3 所需依赖

`package.xml` 中已声明：

- `roscpp`
- `sensor_msgs`、`std_msgs`、`geometry_msgs`
- `cv_bridge`、`image_transport`
- `message_generation`、`message_runtime`

系统依赖：

- OpenCV 3.2+（ROS Melodic 自带）
- Eigen3（后续 Phase 使用）

### 3.4 运行前提

- 输入彩色图像话题已存在。
- 图像编码为 `BGR8`（其他编码可能需要在 `convertToCvImage` 中适配）。

---

## 4. 参数与流程映射

| 参数 | 所在文件 | 作用 |
|------|---------|------|
| `input_topic` | `launch/color_detector.launch` | 输入彩色图像话题 |
| `h_min` / `h_max` | `config/color_thresholds.yaml` | HSV 色调范围（OpenCV 中 H 为 0~179） |
| `s_min` / `s_max` | `config/color_thresholds.yaml` | HSV 饱和度范围 |
| `v_min` / `v_max` | `config/color_thresholds.yaml` | HSV 亮度范围 |
| `morph_kernel_size` | `config/color_thresholds.yaml` | 形态学核大小，0 表示关闭 |
| `min_contour_area` | `config/color_thresholds.yaml` | 轮廓最小面积阈值（像素²） |
| `max_contour_points` | `config/color_thresholds.yaml` | 每个轮廓最多发布的点数 |
| `process_rate` | `config/color_thresholds.yaml` | 处理频率（Hz） |

---

## 5. 面向开发者的重要约定

- **按职责分层**：
  - `include/gemini_geometry_detector/`：类声明
  - `src/`：算法实现与 ROS 节点入口
  - `config/`：参数配置
  - `launch/`：启动文件
  - `rviz/`：RViz 配置文件
  - `msg/`：自定义 ROS 消息
  - `docs/`：算法说明文档
- **算法与 ROS 解耦**：
  - `ColorRegionDetector` 负责纯图像处理逻辑。
  - `color_region_detector_node.cpp` 只负责 ROS 初始化和 `ros::spin()`。
- **内部类型与 ROS msg 解耦**：
  - 内部使用 `cv::Mat` / `std::vector<cv::Point>`。
  - ROS 输出使用自定义 `ContourInfo` / `ContourArray`。
- **消息转换集中在 `publishResults()`**：
  - `cv_bridge` 转换、时间戳同步都统一在该函数完成。
- **每帧 INFO 日志**：
  - 输出图像尺寸、原始轮廓数、有效轮廓数、被过滤轮廓数，便于调参。

---

## 6. 快速开始

### 6.1 编译

```bash
cd /home/jiancui1804/orbbec_ws
source /opt/ros/melodic/setup.bash
catkin_make
```

### 6.2 启动检测节点

确保输入图像话题已经发布，例如 `/camera/color/image_raw`：

```bash
source /home/jiancui1804/orbbec_ws/devel/setup.bash
roslaunch gemini_geometry_detector color_detector.launch
```

如果输入话题不同，可以覆盖：

```bash
roslaunch gemini_geometry_detector color_detector.launch input_topic:=/my_camera/image_raw
```

### 6.3 启动检测节点 + RViz

```bash
source /home/jiancui1804/orbbec_ws/devel/setup.bash
roslaunch gemini_geometry_detector color_detector_rviz.launch
```

### 6.4 可视化

```bash
# 查看标注图
rqt_image_view /gemini_geometry_detector/color/annotated

# 查看二值掩码
rqt_image_view /gemini_geometry_detector/color/mask

# 查看轮廓数值
rostopic echo /gemini_geometry_detector/color/contours
```

---

## 7. RViz 验证

启动 `color_detector_rviz.launch` 后，RViz 会自动显示两个 Image 显示：

1. **Annotated**：带绿色轮廓、红色中心点、蓝色包围盒的标注图
2. **Mask**：HSV 阈值分割后的二值掩码

也可以手动启动：

```bash
rosrun image_view image_view image:=/gemini_geometry_detector/color/annotated
```

正常时应能看到：
- 绿色线条：检测到的目标颜色区域轮廓
- 红色圆点：轮廓中心
- 蓝色矩形：轮廓包围盒

RViz 配置文件位置：`rviz/color_detector.rviz`。
Fixed Frame 默认为 `camera_color_optical_frame`，与输入图像的 `frame_id` 一致；若你的相机使用其他 `frame_id`，请在 RViz 中相应修改。

---

## 8. 参数调参指南

针对黄色地面线的默认配置：

```yaml
h_min: 20
h_max: 40
s_min: 80
s_max: 255
v_min: 80
v_max: 255
morph_kernel_size: 5
min_contour_area: 200
max_contour_points: 64
process_rate: 10.0
```

调参建议：

- **颜色偏色**：调整 `h_min` / `h_max`，黄色通常在 H=15~45。
- **环境同色噪声**：提高 `s_min` 和 `v_min`，让颜色更纯更亮。
- **目标发暗**：降低 `v_min`。
- **掩码噪点多**：增大 `morph_kernel_size`。
- **误检小区域**：增大 `min_contour_area`。
- **远处目标漏检**：减小 `min_contour_area`。

---

## 9. 后续扩展

### Phase 2：加入深度验证

- 订阅深度图和相机内参（如 `/camera/depth/image_raw` 和 `/camera/color/camera_info`）。
- 用 `message_filters` 同步 RGB 与 Depth。
- 对检测到的轮廓中心 `(u, v)`，查询对应深度 `Z`。
- 用相机内参 `fx, fy, cx, cy` 将像素坐标转换到相机三维坐标：

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth(u, v)
```

### Phase 3：地图匹配与距离计算

- 订阅 `/tf`，把相机坐标系下的轮廓转换到 `base_link` / `map`。
- 与地图中的线段做匹配，计算到线距离。

---

## 10. 关键源码索引

| 文件 | 作用 |
|---|---|
| `src/color_region_detector_node.cpp` | ROS 节点入口 |
| `include/gemini_geometry_detector/color_region_detector.h` | `ColorRegionDetector` 类声明 |
| `src/color_region_detector.cpp` | 核心算法实现 |
| `config/color_thresholds.yaml` | HSV 与形态学参数配置 |
| `launch/color_detector.launch` | 单独启动检测节点 |
| `launch/color_detector_rviz.launch` | 启动检测节点 + RViz |
| `rviz/color_detector.rviz` | RViz 配置文件 |
| `msg/ContourInfo.msg` | 单个轮廓消息定义 |
| `msg/ContourArray.msg` | 轮廓数组消息定义 |
| `docs/color_region_detector.md` | 详细算法说明文档 |
