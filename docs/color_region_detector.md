# color_region_detector 算法逻辑说明

本文档结合源码，梳理 `color_region_detector` 从 RGB-D 图像输入到 3D 轮廓输出的完整算法流程，并对每个关键逻辑节点进行说明。

---

## 1. 整体流程

```text
输入数据
    │  ├─ /camera/color/image_raw   (sensor_msgs/Image)
    │  ├─ /camera/depth/image_raw   (sensor_msgs/Image)
    │  └─ /camera/color/camera_info (sensor_msgs/CameraInfo)
    ▼
[RGB-D Sync] message_filters::ApproximateTime 同步
    │
    ▼
[ColorRegionDetector::rgbDepthInfoCallback]
    │  ├─ cv_bridge 解码 RGB 和 Depth
    │  ├─ BGR → HSV 颜色空间转换
    │  ├─ HSV 阈值分割 → 二值掩码
    │  ├─ 形态学开/闭运算去噪
    │  ├─ findContours 提取外轮廓
    │  ├─ 面积过滤 + 计算 2D 中心/包围盒
    │  ├─ DepthProjector::projectContour
    │  │    ├─ 查中心点深度 Z
    │  │    ├─ 用 fx/fy/cx/cy 投影到 3D
    │  │    └─ 计算 mean/min/max depth
    │  └─ 绘制标注图并发布
    ▼
[输出]
    │  ├─ /gemini_geometry_detector/color/mask
    │  ├─ /gemini_geometry_detector/color/annotated
    │  ├─ /gemini_geometry_detector/color/contours
    │  └─ /gemini_geometry_detector/color/contours_3d
    ▼
输出
```

ROS 入口：
- `src/color_region_detector_node.cpp`
- 主类：`ColorRegionDetector`（`include/gemini_geometry_detector/color_region_detector.h`）
- 3D 投影类：`DepthProjector`（`include/gemini_geometry_detector/depth_projector.h`）

---

## 2. 坐标系约定

图像坐标系采用 OpenCV 默认约定：

| 轴 | 方向 | 说明 |
|---|---|---|
| u (x) | 右 | 图像水平方向，单位：像素 |
| v (y) | 下 | 图像垂直方向，单位：像素 |
| 原点 | 左上角 | `(u=0, v=0)` 为图像左上角 |

因此：
- `center.x` 表示轮廓中心在图像中的水平像素坐标。
- `center.y` 表示轮廓中心在图像中的垂直像素坐标。
- `bbox_tl` / `bbox_br` 分别表示包围盒的左上角和右下角像素坐标。

注意：这是**二维图像坐标系**。Phase 2 已经结合 `camera_info` 和深度图把轮廓中心/采样点转换到 `camera_color_optical_frame` 三维坐标系；Phase 3 会进一步转换到 `base_link` / `map`。

---

## 3. 输入与消息解码

### 3.1 订阅输入数据

源码：`src/color_region_detector.cpp` → `ColorRegionDetector::ColorRegionDetector()`

```cpp
rgb_sub_filter_.subscribe(it_, input_topic_, 1);
depth_sub_.subscribe(nh_, depth_topic_, 1);
info_sub_.subscribe(nh_, camera_info_topic_, 1);

sync_.reset(new message_filters::Synchronizer<SyncPolicy>(
    SyncPolicy(10), rgb_sub_filter_, depth_sub_, info_sub_));
sync_->registerCallback(
    boost::bind(&ColorRegionDetector::rgbDepthInfoCallback, this, _1, _2, _3));
```

订阅 RGB-D 同步输入：

| 数据 | 默认话题 | 类型 |
|---|---|---|
| RGB | `/camera/color/image_raw` | `sensor_msgs/Image` |
| Depth | `/camera/depth/image_raw` | `sensor_msgs/Image` |
| CameraInfo | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` |

### 3.2 RGB-D 同步回调

源码：`src/color_region_detector.cpp` → `rgbDepthInfoCallback()`

```cpp
void ColorRegionDetector::rgbDepthInfoCallback(
    const sensor_msgs::ImageConstPtr& rgb_msg,
    const sensor_msgs::ImageConstPtr& depth_msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  cv_rgb = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
  cv_depth = cv_bridge::toCvShare(depth_msg);
  processFrame(rgb_msg, cv_rgb->image, cv_depth->image, info_msg);
}
```

**逻辑说明**：
- 使用 `message_filters::sync_policies::ApproximateTime` 同步 RGB、Depth、CameraInfo。
- 同步后统一处理一帧数据，保证 RGB 像素和 Depth 像素对应。
- 如果 RGB 和 Depth 分辨率不一致，会打印 WARN。

### 3.3 cv_bridge 解码

源码：`src/color_region_detector.cpp` → `rgbDepthInfoCallback()`

```cpp
cv_rgb = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
cv_depth = cv_bridge::toCvShare(depth_msg);
```

**逻辑说明**：
- RGB 期望编码为 `BGR8`。
- Depth 支持 `16UC1`（毫米）和 `32FC1`（米），由 `DepthProjector` 自动判断。
- 期望输入编码为 `BGR8`；如果编码不匹配会抛出异常并打印 ERROR。

---

## 4. HSV 颜色分割

源码：`src/color_region_detector.cpp` → `createColorMask()`

```cpp
cv::Mat ColorRegionDetector::createColorMask(const cv::Mat& bgr_image)
{
  cv::Mat hsv;
  cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(h_min_, s_min_, v_min_), cv::Scalar(h_max_, s_max_, v_max_), mask);
  return mask;
}
```

**逻辑说明**：
1. 把 BGR 图像转换到 HSV 颜色空间。
2. 用 `cv::inRange` 对每个像素做阈值判断：
   - `H ∈ [h_min, h_max]`
   - `S ∈ [s_min, s_max]`
   - `V ∈ [v_min, v_max]`
3. 满足条件的像素在掩码中置 255（白），其余置 0（黑）。

默认配置（黄色地面线）：

```yaml
h_min: 20
h_max: 40
s_min: 80
s_max: 255
v_min: 80
v_max: 255
```

> 注：OpenCV 中 H 通道范围是 0~179，S/V 范围是 0~255。

---

## 5. 形态学处理

源码：`src/color_region_detector.cpp` → `applyMorphology()`

```cpp
void ColorRegionDetector::applyMorphology(cv::Mat& mask)
{
  if (morph_kernel_size_ <= 0)
  {
    return;
  }

  cv::Mat element = cv::getStructuringElement(
      cv::MORPH_ELLIPSE, cv::Size(morph_kernel_size_, morph_kernel_size_));
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);
}
```

**逻辑说明**：
- 使用椭圆结构元素，核大小由 `morph_kernel_size` 指定（默认 5×5，必须是正奇数）。
- 先进行**开运算**（Open）：先腐蚀后膨胀，用于去除小的噪点。
- 再进行**闭运算**（Close）：先膨胀后腐蚀，用于填补目标区域内部的小孔洞。
- 如果 `morph_kernel_size` 设为 0，则跳过形态学处理。

---

## 6. 轮廓检测与过滤

### 6.1 提取外轮廓

源码：`src/color_region_detector.cpp` → `detectContours()`

```cpp
void ColorRegionDetector::detectContours(const cv::Mat& mask,
                                         std::vector<std::vector<cv::Point>>& contours)
{
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
}
```

**逻辑说明**：
- 只检测最外层轮廓（`RETR_EXTERNAL`），不处理轮廓嵌套。
- 使用 `CHAIN_APPROX_SIMPLE` 压缩水平、垂直、对角线段，减少轮廓点数量。

### 6.2 轮廓信息构建与面积过滤

源码：`src/color_region_detector.cpp` → `buildContourInfo()`

```cpp
bool ColorRegionDetector::buildContourInfo(const std::vector<cv::Point>& contour,
                                           int id,
                                           ContourInfo& info)
{
  double area = cv::contourArea(contour);
  if (area < min_contour_area_)
  {
    return false;
  }

  cv::Moments m = cv::moments(contour);
  cv::Point2f center(0, 0);
  if (m.m00 > 0)
  {
    center.x = static_cast<float>(m.m10 / m.m00);
    center.y = static_cast<float>(m.m01 / m.m00);
  }

  cv::Rect bbox = cv::boundingRect(contour);

  info.id = id;
  info.center.x = center.x;
  info.center.y = center.y;
  info.area = area;
  info.bbox_tl.x = bbox.x;
  info.bbox_tl.y = bbox.y;
  info.bbox_br.x = bbox.x + bbox.width;
  info.bbox_br.y = bbox.y + bbox.height;
  // ... 下采样轮廓点
  return true;
}
```

**逻辑说明**：
1. 计算轮廓面积 `cv::contourArea`。
2. 若面积小于 `min_contour_area`（默认 200 像素），直接丢弃。
3. 通过图像矩 `cv::moments` 计算轮廓中心（一阶矩 / 零阶矩）。
4. 计算轴对齐包围盒 `cv::boundingRect`。
5. 对轮廓点进行下采样（默认最多保留 64 个点），减少消息体积。

`ContourInfo` 消息字段：

| 字段 | 类型 | 说明 |
|---|---|---|
| `id` | `int32` | 轮廓编号 |
| `center` | `geometry_msgs/Point` | 轮廓中心像素坐标 `(u, v, 0)` |
| `area` | `float64` | 轮廓面积（像素²） |
| `bbox_tl` | `geometry_msgs/Point` | 包围盒左上角 `(u, v, 0)` |
| `bbox_br` | `geometry_msgs/Point` | 包围盒右下角 `(u, v, 0)` |
| `points` | `geometry_msgs/Point32[]` | 下采样后的轮廓点序列 |

### 6.3 主处理流程中的统计

源码：`src/color_region_detector.cpp` → `processFrame()`

```cpp
int filtered_count = 0;
int depth_filtered_count = 0;
for (const auto& contour : contours)
{
  ContourInfo info_2d;
  if (!buildContourInfo(contour, id, info_2d))
  {
    ++filtered_count;
    continue;
  }

  drawContourOnImage(annotated, contour, info_2d);
  contour_array.contours.push_back(info_2d);

  Contour3DInfo info_3d;
  info_3d.id = info_2d.id;
  // ... copy 2D fields
  if (depth_projector_.projectContour(contour, depth_image, info_3d,
                                      sample_3d_step_, publish_points_3d_))
  {
    contour_3d_array.contours.push_back(info_3d);
  }
  else
  {
    ++depth_filtered_count;
  }
}
```

**逻辑说明**：
- 遍历所有外轮廓，过滤掉小面积轮廓。
- 对每个有效 2D 轮廓调用 `DepthProjector` 投影到 3D。
- 深度无效（超出范围或中心点无深度）的轮廓被过滤。

### 6.4 3D 投影（DepthProjector）

源码：`src/depth_projector.cpp` → `DepthProjector::projectContour()` / `projectPixel()`

#### 6.4.1 相机内参设置

```cpp
void DepthProjector::setCameraInfo(const sensor_msgs::CameraInfoConstPtr& camera_info)
{
  fx_ = camera_info->K[0];
  fy_ = camera_info->K[4];
  cx_ = camera_info->K[2];
  cy_ = camera_info->K[5];
}
```

#### 6.4.2 深度值转换

```cpp
bool DepthProjector::convertDepthValue(const cv::Mat& depth_image, int u, int v, float& depth_m) const
{
  if (depth_image.type() == CV_16UC1)
  {
    uint16_t raw = depth_image.at<uint16_t>(v, u);
    depth_m = static_cast<float>(raw) * depth_scale_;
  }
  else if (depth_image.type() == CV_32FC1)
  {
    depth_m = depth_image.at<float>(v, u);
  }
  return isDepthValid(depth_m);
}
```

#### 6.4.3 单像素投影

```cpp
bool DepthProjector::projectPixel(int u, int v, const cv::Mat& depth_image,
                                  float& X, float& Y, float& Z) const
{
  if (!convertDepthValue(depth_image, u, v, Z)) return false;
  X = (u - cx_) * Z / fx_;
  Y = (v - cy_) * Z / fy_;
  return true;
}
```

#### 6.4.4 轮廓投影

```cpp
bool DepthProjector::projectContour(const std::vector<cv::Point>& contour_2d,
                                    const cv::Mat& depth_image,
                                    Contour3DInfo& info,
                                    int sample_step,
                                    bool publish_points_3d) const
{
  // 1. 投影中心点
  // 2. 采样轮廓点并投影
  // 3. 计算 mean/min/max depth
}
```

**逻辑说明**：
- 先用图像矩计算 2D 中心 `(u, v)`，查询深度后投影到 3D。
- 按 `sample_step` 采样轮廓点，逐一投影。
- 过滤掉深度不在 `[min_depth_m, max_depth_m]` 范围内的轮廓。

#### 6.4.5 投影公式

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth(u, v)
```

坐标系为 `camera_color_optical_frame`，单位：米。

---

## 7. 可视化与发布

### 7.1 在图像上绘制结果

源码：`src/color_region_detector.cpp` → `drawContourOnImage()`

```cpp
void ColorRegionDetector::drawContourOnImage(cv::Mat& annotated,
                                             const std::vector<cv::Point>& contour,
                                             const ContourInfo& info)
{
  cv::Rect bbox(...);
  cv::drawContours(annotated, std::vector<std::vector<cv::Point>>{contour}, -1, cv::Scalar(0, 255, 0), 2);
  cv::circle(annotated, cv::Point(...), 4, cv::Scalar(0, 0, 255), -1);
  cv::rectangle(annotated, bbox, cv::Scalar(255, 0, 0), 2);
}
```

**逻辑说明**：
- 绿色线条：轮廓边界。
- 红色圆点：轮廓中心。
- 蓝色矩形：轴对齐包围盒。

### 7.2 发布结果

源码：`src/color_region_detector.cpp` → `publishResults()`

```cpp
void ColorRegionDetector::publishResults(const std_msgs::Header& header,
                                         const cv::Mat& mask,
                                         const cv::Mat& annotated,
                                         const ContourArray& contour_array,
                                         const Contour3DArray& contour_3d_array)
{
  // mask
  cv_bridge::CvImage mask_msg;
  mask_msg.header = header;
  mask_msg.encoding = sensor_msgs::image_encodings::MONO8;
  mask_msg.image = mask;
  mask_pub_.publish(mask_msg.toImageMsg());

  // annotated image
  cv_bridge::CvImage annotated_msg;
  annotated_msg.header = header;
  annotated_msg.encoding = sensor_msgs::image_encodings::BGR8;
  annotated_msg.image = annotated;
  annotated_pub_.publish(annotated_msg.toImageMsg());

  // 2D contours
  contours_pub_.publish(contour_array);

  // 3D contours
  contours_3d_pub_.publish(contour_3d_array);
}
```

**逻辑说明**：
- 掩码和标注图像使用 `cv_bridge` 重新编码为 ROS 图像消息。
- 2D 和 3D 轮廓消息共享同一 `header`，保证时间戳和 `frame_id` 与输入 RGB 图像一致。

---

## 8. 输出话题

| Topic | 类型 | 说明 |
|---|---|---|
| `/gemini_geometry_detector/color/mask` | `sensor_msgs/Image` | 二值掩码（MONO8） |
| `/gemini_geometry_detector/color/annotated` | `sensor_msgs/Image` | 带轮廓/中心/包围盒的标注图（BGR8） |
| `/gemini_geometry_detector/color/contours` | `gemini_geometry_detector/ContourArray` | 2D 轮廓数组 |
| `/gemini_geometry_detector/color/contours_3d` | `gemini_geometry_detector/Contour3DArray` | 3D 轮廓数组 |

---

## 9. 参数说明

参数文件：`config/detector.yaml`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `input_topic` | `/camera/color/image_raw` | RGB 输入话题 |
| `depth_topic` | `/camera/depth/image_raw` | 深度输入话题 |
| `camera_info_topic` | `/camera/color/camera_info` | 相机内参话题 |
| `h_min` / `h_max` | 20 / 40 | HSV 色调范围 |
| `s_min` / `s_max` | 80 / 255 | HSV 饱和度范围 |
| `v_min` / `v_max` | 80 / 255 | HSV 亮度范围 |
| `morph_kernel_size` | 5 | 形态学核大小，0 表示关闭 |
| `min_contour_area` | 200 | 轮廓最小面积阈值（像素²） |
| `max_contour_points` | 64 | 2D 轮廓最大点数 |
| `sample_3d_step` | 5 | 3D 投影采样步长 |
| `publish_points_3d` | `true` | 是否发布 3D 轮廓点 |
| `depth_scale` | 0.001 | 16UC1 深度转米比例 |
| `min_depth_m` / `max_depth_m` | 0.1 / 10.0 | 有效深度范围 |

---

## 10. 调参指南

### 10.1 HSV 阈值调节

- 目标颜色偏色 → 调整 `h_min` / `h_max`。
  - 黄色通常在 H=15~45 之间。
  - 如果光线偏暖，黄色可能向橙色偏移，需要适当提高 `h_max`。
- 环境中有同色噪声 → 提高 `s_min` 和 `v_min`，让颜色更“纯”、更亮才被认为是目标。
- 目标区域发暗 → 降低 `v_min`。

### 10.2 形态学调节

- 掩码有很多小白点噪点 → 增大 `morph_kernel_size` 或保持开运算。
- 目标区域内部有黑色空洞 → 保持闭运算，可适度增大核。
- 目标本身很小 → 不要设置太大的核，以免把目标腐蚀掉。

### 10.3 面积过滤调节

- 误检太多小区域 → 增大 `min_contour_area`。
- 目标远处变小导致漏检 → 减小 `min_contour_area`。



---

## 11. 关键源码索引

| 文件 | 作用 |
|---|---|
| `src/color_region_detector_node.cpp` | ROS 节点入口 |
| `include/gemini_geometry_detector/color_region_detector.h` | `ColorRegionDetector` 类声明 |
| `src/color_region_detector.cpp` | RGB-D 同步、颜色分割、轮廓检测、发布 |
| `include/gemini_geometry_detector/depth_projector.h` | `DepthProjector` 类声明 |
| `src/depth_projector.cpp` | 深度解析、2D→3D 投影 |
| `config/detector.yaml` | HSV / 形态学 / 深度 / 3D 参数配置 |
| `launch/color_detector.launch` | 单独启动检测节点 |
| `launch/color_detector_rviz.launch` | 启动检测节点 + RViz |
| `rviz/color_detector.rviz` | RViz 配置文件 |
| `msg/ContourInfo.msg` / `ContourArray.msg` | 2D 轮廓消息 |
| `msg/Contour3DInfo.msg` / `Contour3DArray.msg` | 3D 轮廓消息 |

---

## 12. 后续扩展提示

Phase 3 可进一步：
- 订阅 `/tf`，把 `camera_color_optical_frame` 下的 3D 轮廓转换到 `base_link` / `map`。
- 对 3D 轮廓点做 PCA/RANSAC 拟合 3D 直线或 2D 地面线。
- 与地图线匹配，计算车体到目标线的横向距离。

核心公式已经具备：

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth(u, v)
```
