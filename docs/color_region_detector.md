# color_region_detector 算法逻辑说明

本文档结合源码，梳理 `color_region_detector` 从 RGB 图像输入到颜色区域轮廓输出的完整算法流程，并对每个关键逻辑节点进行说明。

---

## 1. 整体流程

```text
输入图像 (sensor_msgs/Image)
    │
    ▼
[ImageSubscriber] cv_bridge 解码为 cv::Mat (BGR)
    │
    ▼
[ColorRegionDetector::processLatestImage]
    │  ├─ 取最新帧 (getLatestImage)
    │  ├─ BGR → HSV 颜色空间转换
    │  ├─ HSV 阈值分割 → 二值掩码
    │  ├─ 形态学开/闭运算去噪
    │  ├─ findContours 提取外轮廓
    │  ├─ 面积过滤 + 计算中心/包围盒
    │  └─ 绘制标注图并发布
    ▼
[输出]
    │  ├─ /gemini_geometry_detector/color/mask
    │  ├─ /gemini_geometry_detector/color/annotated
    │  └─ /gemini_geometry_detector/color/contours
    ▼
输出
```

ROS 入口：
- `src/color_region_detector_node.cpp`
- 主类：`ColorRegionDetector`（`include/gemini_geometry_detector/color_region_detector.h`）

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

注意：这是**二维图像坐标系**，后续 Phase 2/3 会结合 `camera_info` 和深度图转换到相机/车体三维坐标系。

---

## 3. 输入与消息解码

### 3.1 订阅输入图像

源码：`src/color_region_detector.cpp` → `ColorRegionDetector::ColorRegionDetector()`

```cpp
image_sub_ = it_.subscribe(input_topic_, 1, &ColorRegionDetector::imageCallback, this);
```

订阅输入彩色图像：

- 默认话题：`/camera/color/image_raw`
- 消息类型：`sensor_msgs/Image`
- 编码：通常为 `BGR8`
- 来源：相机驱动或 ROS bag

### 3.2 最新帧缓冲

源码：`src/color_region_detector.cpp` → `imageCallback()` / `getLatestImage()`

```cpp
void ColorRegionDetector::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(image_mutex_);
  latest_image_ = msg;
  has_image_ = true;
}
```

```cpp
sensor_msgs::ImageConstPtr ColorRegionDetector::getLatestImage()
{
  std::lock_guard<std::mutex> lock(image_mutex_);
  if (!has_image_)
  {
    ROS_WARN_THROTTLE(5.0, "No color image received yet on topic: %s", input_topic_.c_str());
    return sensor_msgs::ImageConstPtr();
  }
  return latest_image_;
}
```

**逻辑说明**：
- 图像回调只负责把最新帧放入缓冲，不做任何计算。
- 主处理以固定频率（默认 10Hz）通过 `getLatestImage()` 取走最新帧，避免阻塞高频图像流。
- 还没有收到图像时，每 5 秒打印一次 WARN 提示。

### 3.3 cv_bridge 解码

源码：`src/color_region_detector.cpp` → `convertToCvImage()`

```cpp
cv_bridge::CvImageConstPtr ColorRegionDetector::convertToCvImage(const sensor_msgs::ImageConstPtr& image_msg)
{
  try
  {
    return cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return cv_bridge::CvImageConstPtr();
  }
}
```

**逻辑说明**：
- 使用 `cv_bridge::toCvShare` 零拷贝地把 ROS 图像消息转成 `cv::Mat`。
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

源码：`src/color_region_detector.cpp` → `processLatestImage()`

```cpp
int filtered_count = 0;
for (const auto& contour : contours)
{
  ContourInfo info;
  if (buildContourInfo(contour, id++, info))
  {
    drawContourOnImage(annotated, contour, info);
    contour_array.contours.push_back(info);
  }
  else
  {
    ++filtered_count;
  }
}

ROS_DEBUG("Image %dx%d, raw contours: %zu, valid: %zu, filtered by area: %d",
          cv_ptr->image.cols, cv_ptr->image.rows,
          contours.size(), contour_array.contours.size(), filtered_count);
```

**逻辑说明**：
- 遍历所有外轮廓，过滤掉小面积轮廓。
- 每帧打印 DEBUG 日志，包含图像尺寸、原始轮廓数、有效轮廓数、被面积过滤掉的轮廓数。

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
                                         const ContourArray& contour_array)
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

  // contours
  contours_pub_.publish(contour_array);
}
```

**逻辑说明**：
- 掩码和标注图像使用 `cv_bridge` 重新编码为 ROS 图像消息。
- 所有输出消息共享同一 `header`，保证时间戳和 `frame_id` 与输入图像一致。

---

## 8. 输出话题

| Topic | 类型 | 说明 |
|---|---|---|
| `/gemini_geometry_detector/color/mask` | `sensor_msgs/Image` | 二值掩码（MONO8） |
| `/gemini_geometry_detector/color/annotated` | `sensor_msgs/Image` | 带轮廓/中心/包围盒的标注图（BGR8） |
| `/gemini_geometry_detector/color/contours` | `gemini_geometry_detector/ContourArray` | 检测到的轮廓数组 |

---

## 9. 参数说明

参数文件：`config/color_thresholds.yaml`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `h_min` / `h_max` | 20 / 40 | HSV 色调范围（OpenCV 中 H 为 0~179） |
| `s_min` / `s_max` | 80 / 255 | HSV 饱和度范围 |
| `v_min` / `v_max` | 80 / 255 | HSV 亮度范围 |
| `morph_kernel_size` | 5 | 形态学核大小，0 表示关闭，正奇数 |
| `min_contour_area` | 200 | 轮廓最小面积阈值（像素²） |
| `max_contour_points` | 64 | 每个轮廓最多发布的点数，0 表示全部 |
| `process_rate` | 10.0 | 处理频率（Hz） |

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

### 10.4 处理频率

- 默认 10Hz 已经能满足大部分场景。
- 如果需要更低延迟，可提高 `process_rate`，但会占用更多 CPU。

---

## 11. 关键源码索引

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

---

## 12. 后续扩展提示

Phase 2 可在当前节点基础上增加：
- 订阅 `/camera/depth/image_raw` 和 `/camera/color/camera_info`。
- 用 `message_filters` 同步 RGB 与 Depth。
- 对检测到的轮廓中心 `(u, v)`，查询对应深度 `Z`。
- 用相机内参 `fx, fy, cx, cy` 将像素坐标转换到相机三维坐标：

```text
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = depth(u, v)
```

Phase 3 可进一步把结果转换到 `base_link` / `map` 坐标系，并计算到地图线的距离。
