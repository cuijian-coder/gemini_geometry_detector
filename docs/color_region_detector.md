> **注意**：本文档描述当前可插拔引导线估计版本。默认使用 `DepthGuideLineEstimator`（地平面 3D 拟合），`RectangleGuideLineEstimator`（地平面 OBB 矩形中心轴）作为抖动/不规则场景备选，同时保留 `FitLineGuideLineEstimator`（RGB-only 图像平面拟合）。Depth 模式所需的地面平面来源通过 `IGroundPlaneProvider` 接口解耦，支持话题订阅或 IMU 重力估计。

# color_region_detector 算法逻辑说明

本文档结合源码，梳理 `color_region_detector` 从 RGB 图像输入到引导线误差的完整算法流程。引导线估计算法通过 `IGuideLineEstimator` 接口解耦，地面平面来源通过 `IGroundPlaneProvider` 接口解耦。

---

## 1. 整体流程

```text
输入数据
    │  ├─ /camera/color/image_raw          (sensor_msgs/Image)
    │  ├─ /camera/color/camera_info        (sensor_msgs/CameraInfo)
    │  ├─ /tf                              (base_link → camera_color_optical_frame)
    │  ├─ /ground_plane/coefficients       (ground_plane_calibrator/PlaneCoefficients, topic provider)
    │  └─ /camera/gyro_accel/sample        (sensor_msgs/Imu, imu provider)
    ▼
[ColorRegionDetector::rgbCallback]
    │  ├─ 根据 guide_line_estimator_type 创建 IGuideLineEstimator
    │  ├─ 根据 ground_plane_provider_type 创建 IGroundPlaneProvider（Depth 模式）
    │  ├─ cv_bridge 解码 RGB
    │  ├─ BGR → HSV 颜色空间转换
    │  ├─ HSV 阈值分割 → 二值掩码
    │  ├─ 形态学开/闭运算去噪
    │  ├─ findContours 提取外轮廓
    │  ├─ 长度过滤 + 长宽比过滤 + 计算 2D 中心/包围盒
    │  ├─ 自下而上对轮廓分组（最近优先）
    │  ├─ TF 自动计算 target_angle（启动一次）
    │  ├─ 依次调用 IGuideLineEstimator::estimate()，直到成功
    │  │    ├─ FitLine: 归一化图像平面 + cv::fitLine
    │  │    ├─ Depth: 反投影 + 地平面 PCA 直线
    │  │    └─ Rectangle: 反投影 + 地平面 OBB 矩形中心轴
    │  ├─ 计算 yaw_error / lateral_error_n / lateral_error_m
    │  └─ 绘制标注图并发布
    ▼
[输出]
    │  ├─ /gemini_geometry_detector/color/mask
    │  ├─ /gemini_geometry_detector/color/annotated
    │  ├─ /gemini_geometry_detector/color/contours
    │  └─ /gemini_geometry_detector/color/guide_line_error
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

归一化图像平面坐标：

```text
x_n = (u - cx) / fx
y_n = (v - cy) / fy
```

- `fx`、`fy`：相机焦距（像素）。
- `cx`、`cy`：主点（像素）。
- 归一化坐标中心为 `(0, 0)`，因此 `lateral_error_n` 直接等于 ROI 行处的 `x_n`。

---

## 3. 输入与消息解码

### 3.1 订阅输入数据

源码：`src/color_region_detector.cpp` → `ColorRegionDetector::ColorRegionDetector()`

```cpp
rgb_sub_ = it_.subscribe(input_topic_, 1,
                         &ColorRegionDetector::rgbCallback, this);
camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1,
                                 &ColorRegionDetector::cameraInfoCallback, this);
```

订阅输入：

| 数据 | 默认话题 | 类型 |
|---|---|---|
| RGB | `/camera/color/image_raw` | `sensor_msgs/Image` |
| CameraInfo | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` |

### 3.2 RGB 回调

源码：`src/color_region_detector.cpp` → `rgbCallback()`

```cpp
void ColorRegionDetector::rgbCallback(const sensor_msgs::ImageConstPtr& rgb_msg)
{
  cv_rgb = cv_bridge::toCvShare(rgb_msg, sensor_msgs::image_encodings::BGR8);
  processFrame(rgb_msg, cv_rgb->image);
}
```

**逻辑说明**：
- 不再同步点云，只接收 RGB 图像。
- RGB 期望编码为 `BGR8`；如果编码不匹配会抛出异常并打印 ERROR。

### 3.3 CameraInfo 回调

源码：`src/color_region_detector.cpp` → `cameraInfoCallback()`

```cpp
void ColorRegionDetector::cameraInfoCallback(
    const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  const auto& k = info_msg->K;
  camera_intrinsics_.fx = static_cast<float>(k[0]);
  camera_intrinsics_.fy = static_cast<float>(k[4]);
  camera_intrinsics_.cx = static_cast<float>(k[2]);
  camera_intrinsics_.cy = static_cast<float>(k[5]);
}
```

**逻辑说明**：
- 缓存原始相机内参。
- 缩放后的图像坐标由估计算法内部自行处理（`FitLineGuideLineEstimator` 忽略内参；`DepthGuideLineEstimator` 在 `configure()` 中按 `image_scale_` 缩放内参）。

---

## 4. HSV 颜色分割

源码：`src/color_region_detector.cpp` → `createColorMask()`

```cpp
cv::Mat ColorRegionDetector::createColorMask(const cv::Mat& bgr_image)
{
  cv::Mat hsv;
  cv::cvtColor(bgr_image, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(h_min_, s_min_, v_min_),
              cv::Scalar(h_max_, s_max_, v_max_), mask);
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

### 5.1 HSV 掩码时间滤波

源码：`src/color_region_detector.cpp` → `processFrame()`

```cpp
if (mask_filter_alpha_ > 0.0)
{
  if (mask_accumulator_.empty() ||
      mask_accumulator_.size() != mask.size() ||
      mask_accumulator_.type() != CV_32F)
  {
    mask_accumulator_ = cv::Mat(mask.size(), CV_32F, cv::Scalar(0.0));
  }

  cv::Mat float_mask;
  mask.convertTo(float_mask, CV_32F, 1.0 / 255.0);
  cv::addWeighted(mask_accumulator_, 1.0 - mask_filter_alpha_,
                  float_mask, mask_filter_alpha_, 0.0,
                  mask_accumulator_);

  mask_accumulator_.convertTo(mask, CV_8U, 255.0);
  cv::threshold(mask, mask, 127, 255, cv::THRESH_BINARY);
}
```

**逻辑说明**：
- 在 HSV 阈值分割之后、形态学处理之前，对二值掩码做指数滑动平均（EMA）。
- 历史掩码权重为 `1 - alpha`，当前帧权重为 `alpha`。
- 当 `alpha = 0.0` 时关闭滤波；`alpha` 越大，当前帧占比越高，响应越快但平滑越弱。
- 对于静止/缓慢变化的相机，推荐 `0.1 ~ 0.3`，可显著抑制 HSV 分割的帧间抖动。
- 建议搭配形态学处理（如 `morph_kernel_size: 5`）进一步去除剩余孤点。

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

### 6.2 轮廓信息构建与长度过滤

源码：`src/color_region_detector.cpp` → `buildContourInfo()`

```cpp
bool ColorRegionDetector::buildContourInfo(const std::vector<cv::Point>& contour,
                                           int id,
                                           ContourInfo& info,
                                           double min_contour_length)
{
  const double length = cv::arcLength(contour, true);
  if (length < min_contour_length)
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
  // ... 填充 ContourInfo
  return true;
}
```

**逻辑说明**：
1. 计算轮廓周长 `cv::arcLength(contour, true)`。
2. 若周长小于 `min_contour_length`（默认 50 像素），直接丢弃。
   - 使用长度而非面积，可以避免细长的水平引导线因面积过小而误过滤。
3. 通过图像矩 `cv::moments` 计算轮廓中心（一阶矩 / 零阶矩）。
4. 计算轴对齐包围盒 `cv::boundingRect`。
5. 计算长宽比 `aspect_ratio = max(width, height) / min(width, height)`。
   - 若小于 `min_aspect_ratio`（默认 1.0，即无过滤），丢弃。
   - 若大于 `max_aspect_ratio`（默认 0.0，表示不启用上限），丢弃。
   - 该过滤用于剔除近似圆形或方形的非线状噪声区域。
6. 对轮廓点进行下采样（默认最多保留 64 个点），减少消息体积。

`ContourInfo` 消息字段：

| 字段 | 类型 | 说明 |
|---|---|---|
| `id` | `int32` | 轮廓编号 |
| `center` | `geometry_msgs/Point` | 轮廓中心像素坐标 `(u, v, 0)` |
| `area` | `float64` | 轮廓面积（像素²） |
| `bbox_tl` | `geometry_msgs/Point` | 包围盒左上角 `(u, v, 0)` |
| `bbox_br` | `geometry_msgs/Point` | 包围盒右下角 `(u, v, 0)` |
| `points` | `geometry_msgs/Point32[]` | 下采样后的轮廓点序列 |

### 6.3 自下而上的锚点合并

源码：`src/color_region_detector.cpp` → `findBottomUpMergeGroups()` / `computeContourFeatures()`

**启用条件**：`enable_contour_merging: true` 时合并同组轮廓；`false` 时每个有效轮廓自成一组。合并需要有效的 `CameraInfo`。

**逻辑说明**：
1. 对**已通过长度和长宽比过滤**的轮廓，分别计算：
   - 归一化图像平面中心 `center_n`。
   - 归一化图像平面方向角 `angle`，由 `cv::fitLine` 在归一化坐标上拟合得到。
   - 轴对齐包围盒 `bbox`（缩放后的像素坐标）。
2. 若合并启用，对每对有效轮廓判断是否满足合并条件：
   - 方向角差异 `angle_diff < merge_max_angle_diff_deg`（默认 15°）。
   - 包围盒最近距离 `region_gap < merge_max_region_gap_n`（默认 0.1，约 60 像素@fx≈613）。该距离通过 `computeNormalizedBboxGap()` 在归一化图像平面上计算：先分别求两个 bbox 在 x/y 方向上的间隙，再取欧氏距离；若 bbox 有重叠则间隙为 0。
3. 使用并查集把满足条件的轮廓归为一组（传递合并）。
4. 所有组合按**包围盒底部 y 坐标从大到小排序**，即**从图像底部向顶部排列**。底部组代表离摄像头最近的引导线，会被优先处理。
5. 后续 `DepthGuideLineEstimator` 按此顺序依次尝试每个组：把组内轮廓像素拼接后反投影到地面，第一个能成功拟合地面的组即被采用。

> 注意：`ContourArray` 仍只发布原始轮廓；最终选中的组合仅通过 `GuideLineError` 和 `annotated` 图中的高亮体现。

---

## 7. 引导线拟合与误差计算

### 7.1 选取候选组

源码：`src/color_region_detector.cpp` → `processFrame()`

**逻辑说明**：
- 不再简单选取“最长”轮廓，而是优先处理**图像底部最近的合并组**。
- 对每组调用 `IGuideLineEstimator::estimate()`，直到某组返回有效结果。
- 若所有组都失败，最终输出 `GuideLineError invalid`。
- 该策略能自动避开地平线、墙面等远处的长条状误检，优先使用离机器人最近的地面引导线。

**底部区域判定**：
- 通过 `roi_bottom_ratio` 控制。
- 只有当组合中所有轮廓的包围盒最低点 `bottom_y >= roi_bottom_ratio * image_height` 时，才被认为是“底部候选组”。
- 底部候选组按 `bottom_y` 从大到小排序（先最近），内部再按轮廓总长度从长到短排序。
- 非底部候选组作为兜底，也按 `bottom_y` 从大到小排序。
- `roi_bottom_ratio = 0.5` 表示只看图像下半部分；`0.0` 表示整图都参与；`0.75` 表示只看最下方四分之一区域。

### 7.2 矩形近似（仅可视化）

源码：`src/color_region_detector.cpp` → `getRectangularContour()`

**逻辑说明**：
- 对 `largest_contour` 调用 `cv::minAreaRect` 得到最小外接矩形。
- 该矩形**仅用于在 annotated 图上可视化**，不参与控制计算。
- 因为透视效应，地面引导线在图像中可能是梯形或不规则四边形，强扭成矩形会引入误差，因此控制使用原始轮廓点直接拟合。

### 7.3 归一化轮廓点

源码：`src/color_region_detector.cpp` → `normalizeContourPoints()`

```cpp
for (const auto& pt : contour)
{
  float x_n = (pt.x - cx) / fx;
  float y_n = (pt.y - cy) / fy;
  normalized_points.emplace_back(x_n, y_n);
}
```

**逻辑说明**：
- 把像素坐标转换到归一化图像平面。
- 如果轮廓点数量超过 `max_contour_points`，则均匀采样。

### 7.4 中心线拟合

源码：`src/color_region_detector.cpp` → `fitCenterLineNormalized()`

```cpp
cv::fitLine(normalized_points, line, cv::DIST_L2, 0, 0.01, 0.01);
```

**逻辑说明**：
- 在归一化图像平面拟合直线，输出 `(vx, vy, x0, y0)`。
- 由于坐标已经归一化，拟合结果对相机分辨率/焦距具有不变性。

### 7.5 横向误差

源码：`src/color_region_detector.cpp` → `computeLateralErrorN()`

```cpp
roi_y_pixel = roi_y_ratio_ * image_rows;
roi_y_n     = (roi_y_pixel - cy) / fy;
x_roi_n     = x0_n + (roi_y_n - y0_n) * vx / vy;
lateral_error_n = x_roi_n;
```

**逻辑说明**：
- 默认在图像下方 75% 处取 ROI 行。
- 在 ROI 行处求中心线 x 坐标，得到归一化横向误差。
- 归一化图像平面中心为 `0`，因此 `lateral_error_n` 直接等于 `x_roi_n`。
- 正值表示引导线在图像中心右侧，负值在左侧。

### 7.6 目标方向角（TF 自动计算）

源码：`src/color_region_detector.cpp` → `computeTargetAngleFromTf()` / `ensureTargetAngleComputed()`

```cpp
geometry_msgs::TransformStamped transform =
    tf_buffer_.lookupTransform(camera_frame_, base_frame_, ros::Time(0), ...);

Eigen::Quaterniond q(transform.transform.rotation.w,
                     transform.transform.rotation.x,
                     transform.transform.rotation.y,
                     transform.transform.rotation.z);
Eigen::Vector3d forward_cam = q * Eigen::Vector3d(1.0, 0.0, 0.0);

target_angle_ = atan2(forward_cam.y() / forward_cam.z(),
                      forward_cam.x() / forward_cam.z());
```

**逻辑说明**：
- 车辆在 `base_link` 中的前进方向为 +X。
- 通过 TF 查询 `base_frame → camera_frame` 的旋转，将该方向投影到相机图像平面。
- 使用归一化图像坐标计算角度，结果与 `fx/fy` 无关。
- 只在启动时计算一次，之后固定使用。
- 如果 TF 查询失败，则回退到 YAML 中配置的 `target_angle`。

### 7.7 方向角误差

源码：`src/fit_line_guide_line_estimator.cpp` → `computeYawError()`

```cpp
double angle = atan2(line[1], line[0]);
double error1 = normalizeAngle(angle - target_angle_);
double error2 = normalizeAngle(angle + M_PI - target_angle_);
double yaw_error = (abs(error1) < abs(error2)) ? error1 : error2;
```

**逻辑说明**：
- 引导线在几何上是**无向**的：`cv::fitLine` 可能返回同一直线的任意一个方向。
- 因此同时计算两个相反方向的误差，取绝对值较小的那个，保证结果稳定。
- `yaw_error` 表示当前引导线方向与目标方向的偏差，归一化到 `[-pi/2, pi/2]`。

### 7.8 DepthGuideLineEstimator 地平面拟合

源码：`src/depth_guide_line_estimator.cpp`

当 `guide_line_estimator_type: "DepthGuideLineEstimator"` 时，算法流程为：

1. **地面平面来源**：`ColorRegionDetector` 创建 `IGroundPlaneProvider` 实现（topic 或 imu），并通过回调把平面传给 `DepthGuideLineEstimator::setGroundPlane(normal, d)`。平面方程为 `normal·P + d = 0`。
2. **反投影**：对 `largest_contour` 的像素点 `(u, v)` 计算归一化射线：
   ```text
   ray = ((u - cx) / fx, (v - cy) / fy, 1)
   ```
3. **地面交点**：射线 `P = t * ray` 与地面平面 `normal·P + d = 0` 求交：
   ```text
   t = -d / (normal · ray)
   P_ground = t * ray
   ```
   只保留 `t > 0`（相机前方）的有效点。
4. **地平面直线拟合**：对有效地面点做 PCA，取最大特征值对应的特征向量作为地平面直线方向。
5. **方向对齐**：将直线方向与车辆前进方向（相机 Z 轴在地平面的投影）对齐，避免方向歧义。
6. **误差计算**：
   ```text
   yaw_error       = atan2(dir · left, dir · forward) - target_angle
   lateral_error_m = signed_distance(look_ahead_reference, ground_line)
   ```
   其中 `look_ahead_reference = -d * normal + look_ahead_m * forward`，即相机光心在地平面的投影再向前 `look_ahead_m` 米。

### 7.9 RectangleGuideLineEstimator 地平面 OBB 拟合

源码：`src/rectangle_guide_line_estimator.cpp`

当 `guide_line_estimator_type: "RectangleGuideLineEstimator"` 时，算法流程与 `DepthGuideLineEstimator` 前 3 步相同（地面平面、反投影、地面交点），区别在第 4 步：

4. **OBB 矩形拟合**：
   - 计算地面点云质心。
   - 对地面点云做 PCA，取最大特征值方向为**长轴**（引导线方向），中间特征值方向为**短轴**（线宽方向）。
   - 用长轴/短轴包围所有地面点，得到 OBB 矩形。
   - 矩形中心作为线上参考点，长轴作为中心轴。
5. **宽度软校验**：比较实际短轴宽度与 `guide_line_width_m`，偏差大时打印 WARN，但不拒绝结果。
6. **方向对齐**：与 Depth 模式相同，对齐车辆前向。
7. **误差计算**：与 Depth 模式相同，使用 `lateral_error_m`。

详细说明见 `docs/rectangle_guide_line_estimator.md`。

### 7.10 IGroundPlaneProvider 地面平面来源

源码：
- `include/gemini_geometry_detector/ground_plane_provider_interface.h`
- `src/topic_ground_plane_provider.cpp`
- `src/imu_ground_plane_provider.cpp`

#### TopicGroundPlaneProvider

- 订阅 `ground_plane_topic`（默认 `/ground_plane/coefficients`）。
- 每次收到 `PlaneCoefficients` 后解包为 `(normal, d)` 并回调给 `DepthGuideLineEstimator` 或 `RectangleGuideLineEstimator`。

#### ImuGroundPlaneProvider

- 订阅 `imu_topic`（默认 `/camera/gyro_accel/sample`）。
- 对 IMU 重力做低通滤波，并按 `imu_to_camera_qx/qy/qz/qw` 旋转到相机坐标系。
- 计算地面法向量 `normal = -gravity.normalized()`，取 `d = camera_height`。
- 每次 IMU 回调都通过回调把平面传给当前使用的 Depth/Rectangle estimator。

**逻辑说明**：
- 控制量直接是车体坐标系下的角度和米制横向偏移，便于下游控制器直接使用。
- `lateral_error_m > 0` 表示引导线在当前前视参考点的右侧（与 `lateral_error_n` 符号约定一致）。

---

## 8. 可视化与发布

### 8.1 在图像上绘制结果

源码：`src/color_region_detector.cpp` → `drawContourOnImage()` / `drawGuideLineErrorOnImage()`

绘制内容：
- 绿色线条：轮廓边界。
- 红色圆点：轮廓中心。
- 蓝色矩形：轴对齐包围盒。
- 黄色多边形：`cv::minAreaRect` 最小外接矩形（仅可视化）。
- 绿色直线：拟合的中心线。
- 黄色水平线：ROI 行。
- 红色圆点：ROI 行与中心线的交点。
- 蓝色圆点：图像中心在 ROI 行的位置。
- 品红色线段：连接图像中心与 ROI 交点，直观表示横向误差。
- 文字：`yaw: X.XXX rad`、`lat: X.XXXX`。

### 8.2 误差统计叠加

源码：`src/color_region_detector.cpp` → `updateErrorStats()` / `drawErrorStatsOverlay()`

当 `enable_error_stats: true` 时，每次得到有效的 `GuideLineError` 后都会更新滑动窗口统计，并在 `annotated` 图像左上角叠加文字：

```text
Error statistics (window=100)
yaw:  3.00 deg
yaw  min/max/avg: -7.07 / 10.75 / 1.98 deg
lat:  0.0234 m
lat  min/max/avg: -0.0543 / 0.0821 / 0.0123 m
```

**逻辑说明**：
- 只统计 `valid = true` 的帧。
- 滑动窗口大小由 `error_stats_window` 控制，默认保存最近 100 帧。
- 横向误差在 `DepthGuideLineEstimator` 模式下使用 `lateral_error_m`，在 `FitLineGuideLineEstimator` 模式下使用 `lateral_error_n`。
- 统计量包括：当前值、最小值、最大值、均值。
- 背景使用黑色半透明矩形，文字为绿色，便于在 RViz 中查看。

### 8.3 发布结果

源码：`src/color_region_detector.cpp` → `publishResults()`

```cpp
void ColorRegionDetector::publishResults(const std_msgs::Header& header,
                                         const cv::Mat& mask,
                                         const cv::Mat& annotated,
                                         const ContourArray& contour_array)
{
  // mask
  // annotated image
  // 2D contours
}
```

额外发布：
- `/gemini_geometry_detector/color/guide_line_error`：
  - `yaw_error`（rad）
  - `lateral_error_n`（归一化图像平面横向误差，FitLine 模式有效）
  - `lateral_error_m`（米制地平面横向误差，Depth 模式有效）
  - `valid`（是否有效）
  - `roi_point`（原始图像坐标中的 ROI 交点或 Depth 模式的前视参考点投影）

---

## 9. 输出话题

| Topic | 类型 | 说明 |
|---|---|---|
| `/gemini_geometry_detector/color/mask` | `sensor_msgs/Image` | 二值掩码（MONO8） |
| `/gemini_geometry_detector/color/annotated` | `sensor_msgs/Image` | 带轮廓、中心线、ROI、误差的标注图（BGR8） |
| `/gemini_geometry_detector/color/contours` | `gemini_geometry_detector/ContourArray` | 2D 轮廓数组 |
| `/gemini_geometry_detector/color/guide_line_error` | `gemini_geometry_detector/GuideLineError` | 引导线角度与横向误差（n/m） |

---

## 10. 参数说明

参数文件：`config/detector.yaml`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `input_topic` | `/camera/color/image_raw` | RGB 输入话题 |
| `camera_info_topic` | `/camera/color/camera_info` | 相机内参话题 |
| `h_min` / `h_max` | 20 / 40 | HSV 色调范围 |
| `s_min` / `s_max` | 80 / 255 | HSV 饱和度范围 |
| `v_min` / `v_max` | 80 / 255 | HSV 亮度范围 |
| `morph_kernel_size` | 5 | 形态学核大小，0 表示关闭 |
| `min_contour_length` | 50 | 轮廓最小周长阈值（像素） |
| `max_contour_points` | 64 | 轮廓最大点数（拟合 + 发布） |
| `min_aspect_ratio` | 1.0 | 轮廓最小长宽比（长边/短边） |
| `max_aspect_ratio` | 0.0 | 轮廓最大长宽比，`<=0` 表示不限制 |
| `enable_contour_merging` | false | 是否启用断裂轮廓合并 |
| `merge_max_angle_diff_deg` | 15.0 | 合并方向角差异阈值（°） |
| `merge_max_region_gap_n` | 0.1 | 合并包围盒最近归一化距离阈值 |
| `mask_filter_alpha` | 0.0 | HSV 掩码时间滤波系数（0.0 关闭；0.1~0.3 推荐） |
| `roi_bottom_ratio` | 0.5 | 轮廓组“底部”判定阈值：包围盒最低点低于该比例图像高度才算底部候选 |
| `enable_error_stats` | true | 是否在 annotated 图上叠加误差统计 |
| `error_stats_window` | 100 | 误差统计滑动窗口帧数 |
| `image_scale` | 0.5 | 颜色检测和引导线拟合的图像缩放比例 |
| `use_tf_target_angle` | true | 是否通过 TF 自动计算 `target_angle` |
| `target_angle` | `0.0` / `-pi/2` | 目标线方向（rad），TF 失败时作为回退；Depth 模式为地平面相对车体前向的角，FitLine 模式为图像平面角 |
| `base_frame` | `base_link` | 车辆坐标系 |
| `camera_frame` | `camera_color_optical_frame` | 相机光心坐标系 |
| `roi_y_ratio` | 0.75 | ROI 行相对图像高度的比例（FitLine 模式） |
| `roi_y` | -1 | ROI 行绝对像素值，`-1` 表示使用 `roi_y_ratio`（FitLine 模式） |
| `guide_line_estimator_type` | `DepthGuideLineEstimator` | 引导线估计算法：`DepthGuideLineEstimator` / `RectangleGuideLineEstimator` / `FitLineGuideLineEstimator` |
| `look_ahead_m` | 0.0 | Depth / Rectangle 模式下前视参考点距离（m） |
| `ground_plane_provider_type` | `topic` | 地面平面来源：`topic` / `imu`（Depth / Rectangle 模式使用） |
| `ground_plane_topic` | `ground_plane/coefficients` | topic provider 订阅话题 |
| `imu_topic` | `/camera/gyro_accel/sample` | imu provider 订阅话题 |
| `camera_height` | 0.8 | imu provider 相机高度（m） |
| `gravity_filter_alpha` | 1.0 | imu provider 重力低通滤波系数 |
| `imu_to_camera_qx/qy/qz/qw` | 0/0/0/1 | imu provider IMU→相机旋转 |
| `guide_line_width_m` | 0.10 | Rectangle 模式下引导线物理宽度（m） |
| `obb_width_tolerance_m` | 0.05 | Rectangle 模式下宽度偏差告警阈值（m） |
| `obb_max_outlier_ratio` | 0.30 | Rectangle 模式下离群点比例告警阈值 |
| `obb_min_ground_points` | 10 | Rectangle 模式下最少有效地面点数 |

---

## 11. 调参指南

### 11.1 HSV 阈值调节

- 目标颜色偏色 → 调整 `h_min` / `h_max`。
  - 黄色通常在 H=15~45 之间。
  - 如果光线偏暖，黄色可能向橙色偏移，需要适当提高 `h_max`。
- 环境中有同色噪声 → 提高 `s_min` 和 `v_min`，让颜色更“纯”、更亮才被认为是目标。
- 目标区域发暗 → 降低 `v_min`。

### 11.2 形态学调节

- 掩码有很多小白点噪点 → 增大 `morph_kernel_size` 或保持开运算。
- 目标区域内部有黑色空洞 → 保持闭运算，可适度增大核。
- 目标本身很小 → 不要设置太大的核，以免把目标腐蚀掉。

### 11.2.1 HSV 掩码时间滤波调节

- 掩码帧间抖动明显 → 增大 `mask_filter_alpha`（越接近 1.0 响应越快）。
- 静止相机下推荐从 `0.15` 开始尝试。
- 相机快速移动时 → 关闭或保持较小值（如 `0.05`），避免历史掩码拖尾。
- 滤波后仍有孤点 → 配合 `morph_kernel_size` 开运算。

### 11.3 长度过滤调节

- 误检太多短线段 → 增大 `min_contour_length`。
- 目标远处变短导致漏检 → 减小 `min_contour_length`。
- 细长水平线被过滤 → 确认 `min_contour_length` 是否过小；当前使用周长而非面积，已经缓解该问题。

### 11.4 轮廓合并参数调节

- 引导线断裂成多段 → 启用 `enable_contour_merging: true`。
- 合并过多、把不同目标连起来 → 减小 `merge_max_region_gap_n` 或收紧 `merge_max_angle_diff_deg`。
- 同一条线有间距但未合并 → 增大 `merge_max_region_gap_n`（该距离同时包含 x/y 方向间隙）。
- 弯道导致两段线方向不一致 → 适当放宽 `merge_max_angle_diff_deg`（但会引入误合并风险）。

### 11.5 引导线参数调节

- 引导线方向不对 → 调整 `target_angle`。
- 横向误差响应过晚 → 增大 `roi_y_ratio`（更靠近图像底部）。
- 横向误差噪声大 → 减小 `roi_y_ratio`（更远离车辆，线更稳定），但响应会滞后。

### 11.6 底部区域阈值调节

- 地平线/远处背景误检被优先选中 → 增大 `roi_bottom_ratio`（如 0.6~0.7），限制只看更靠近底部的区域。
- 引导线在画面中位置偏上、底部组太少 → 减小 `roi_bottom_ratio`（如 0.3~0.4）。
- 想完全按底部远近选择，不限制高度 → 设为 `0.0`。

### 11.7 误差统计叠加调节

- 不需要显示统计 → `enable_error_stats: false`。
- 窗口太短、统计跳动 → 增大 `error_stats_window`（如 200~500）。
- 窗口太长、响应迟钝 → 减小 `error_stats_window`（如 30~50）。

---

## 12. 关键源码索引

| 文件 | 作用 |
|---|---|
| `src/color_region_detector_node.cpp` | ROS 节点入口 |
| `include/gemini_geometry_detector/color_region_detector.h` | `ColorRegionDetector` 类声明 |
| `src/color_region_detector.cpp` | RGB 处理、轮廓检测、算法调度、发布 |
| `src/fit_line_guide_line_estimator.cpp` | RGB 图像平面引导线估计算法实现 |
| `src/depth_guide_line_estimator.cpp` | 地平面 3D 引导线估计算法实现 |
| `src/topic_ground_plane_provider.cpp` | 话题地面平面来源实现 |
| `src/imu_ground_plane_provider.cpp` | IMU 地面平面来源实现 |
| `include/gemini_geometry_detector/guide_line_estimator_interface.h` | 引导线估计算法接口 |
| `include/gemini_geometry_detector/ground_plane_provider_interface.h` | 地面平面来源接口 |
| `include/gemini_geometry_detector/fit_line_guide_line_estimator.h` | RGB 实现类声明 |
| `include/gemini_geometry_detector/depth_guide_line_estimator.h` | 深度实现类声明 |
| `include/gemini_geometry_detector/topic_ground_plane_provider.h` | topic provider 类声明 |
| `include/gemini_geometry_detector/imu_ground_plane_provider.h` | imu provider 类声明 |
| `config/detector.yaml` | HSV / 形态学 / 引导线参数配置 |
| `launch/color_detector.launch` | 单独启动检测节点 |
| `launch/color_detector_rviz.launch` | 启动检测节点 + RViz |
| `rviz/color_detector.rviz` | RViz 配置文件 |
| `msg/ContourInfo.msg` / `ContourArray.msg` | 2D 轮廓消息 |
| `msg/GuideLineError.msg` | 引导线误差消息 |
| `docs/depth_guide_line_estimator.md` | DepthGuideLineEstimator 反投影与误差计算详细说明 |
| `docs/rectangle_guide_line_estimator.md` | RectangleGuideLineEstimator OBB 中心轴详细说明 |

---

## 13. 后续扩展提示

- Depth 模式下，下游控制器可直接使用米制误差：
  ```text
  angular_z = -Kp_yaw * yaw_error - Kp_lateral * lateral_error_m
  ```
- FitLine 模式下，如需车体坐标系控制，可额外订阅 `/tf` 转换图像平面误差。
- 需要新的地面平面来源时，实现 `IGroundPlaneProvider` 接口并在 `ColorRegionDetector` 中注册即可。
- 与地图线匹配，计算车体到目标线的横向距离。
