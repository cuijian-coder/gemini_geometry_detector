# DepthGuideLineEstimator 算法逻辑说明

本文档结合源码，梳理 `DepthGuideLineEstimator` 在 `gemini_geometry_detector` 中的工作机制。它负责把 RGB 图像中的 2D 轮廓反投影到地面平面，再拟合地平面上的引导线并计算车体坐标系下的角度和横向误差。

与 `color_region_detector` 的接口关系可参考 [`docs/color_region_detector.md`](./color_region_detector.md)。

---

## 1. 整体流程

```text
输入
    ├─ 2D 轮廓（来自 ColorRegionDetector，已缩放）
    ├─ 相机内参（CameraIntrinsics，已按 image_scale 缩放）
    ├─ 目标方向 target_angle（rad，地平面相对车体前向）
    ├─ 前视距离 look_ahead_m（m）
    └─ 地平面 (normal, d)  由 IGroundPlaneProvider 提供
    ▼
[DepthGuideLineEstimator::estimate]
    ├─ 对候选轮廓依次尝试
    ├─ 归一化地平面符号（d > 0）
    ├─ 对每个轮廓像素反投影：pixel → ray → ray-plane intersection
    ├─ 丢弃 ray 与地平面平行或交点在相机后方的点（t <= 0）
    ├─ 对有效地面点做 PCA 拟合直线
    ├─ 对齐方向到车体前向
    ├─ 计算 yaw_error / lateral_error_m
    └─ 返回 GuideLineError
    ▼
输出
    ├─ yaw_error（rad）
    ├─ lateral_error_m（m）
    └─ roi_point（地面前视参考点在归一化图像平面的投影）
```

ROS 入口：
- `ColorRegionDetector::processFrame()`（`src/color_region_detector.cpp`）
- 估计器类：`DepthGuideLineEstimator`（`include/gemini_geometry_detector/depth_guide_line_estimator.h`）

---

## 2. 坐标系约定

与 `color_region_detector` 一致，采用 OpenCV 相机光学坐标系：

| 轴 | 方向 | 说明 |
|---|---|---|
| X | 右 | 图像水平方向 |
| Y | 下 | 图像垂直方向 |
| Z | 前 | 相机光轴向前 |
| 原点 | 相机光心 | `(0, 0, 0)` |

归一化图像平面坐标：

```text
x_n = (u - cx) / fx
y_n = (v - cy) / fy
```

地平面方程：

```text
n · P + d = 0
```

- `n`：地平面单位法向量。
- `d`：相机光心到地平面的带符号距离（见第 3 节符号约定）。
- `P`：地平面上的 3D 点。

---

## 3. 地平面符号约定与归一化

### 3.1 符号约定

`DepthGuideLineEstimator` 内部统一使用以下约定：

- `n` 指向**远离地面**的方向（大致朝向相机/天空）。
- `d > 0`，表示相机光心位于 `n` 所指的一侧（即相机在地面上方）。

在此约定下，一条指向地面的射线 `ray` 满足 `n · ray < 0`，于是射线与地平面交点参数：

```text
t = -d / (n · ray) > 0
```

交点位于相机前方。

### 3.2 自动归一化

不同 `IGroundPlaneProvider` 可能以不同符号发布地平面。例如：

- `ImuGroundPlaneProvider`：直接给出 `n` 向上、`d = camera_height`，通常已满足约定。
- `TopicGroundPlaneProvider`：可能从 `ground_plane_calibrator` 收到 `d < 0` 的平面（法向量指向地面）。

为避免下游射线求交失败，`setGroundPlane()` 在收到平面后自动归一化：

```cpp
if (ground_d_ < 0.0f)
{
    ground_normal_ = -ground_normal_;
    ground_d_ = -ground_d_;
}
```

翻转 `(n, d)` 不改变几何平面，但保证 `d > 0` 且 `n` 指向远离地面。

### 3.3 地平线位置

在图像中，地平线对应 `n · ray = 0` 的像素。满足 `n · ray < 0` 的像素位于地平线下方（地面区域），满足 `n · ray > 0` 的像素位于地平线上方（天空/墙面/远处背景）。

实际运行中，如果某个轮廓的所有点都落在地平线以上，所有交点的 `t <= 0`，估计器会返回 `valid = false`。

---

## 4. 自下而上的锚点合并与选择

源码：`src/color_region_detector.cpp` → `ColorRegionDetector::processFrame()` / `findBottomUpMergeGroups()`

### 4.1 为什么从底部开始

按长度/面积选出的“最大轮廓”并不总是地面引导线。常见误检包括：

- 地平线本身
- 墙面与地面交界线
- 货架、柱子等长条状背景亮区

这些轮廓往往很长，但反投影到地面时全部落在相机后方，导致 `DepthGuideLineEstimator` 报告 `not enough ground-plane points`。

真实地面引导线有一个关键特性：**离摄像头越近，在图像中位置越靠下**。因此从图像底部向上搜索，优先处理最下方的线，能天然避开上半部分误检。

### 4.2 锚点合并策略

1. 收集所有通过长度/长宽比过滤的轮廓。
2. 计算每个轮廓的归一化方向、中心、包围盒。
3. 若启用合并（`enable_contour_merging: true`），用并查集按方向+距离条件把兼容轮廓归为一组。
4. 所有组合按**包围盒底部 y 坐标从大到小排序**（底部优先）。
5. 从底部开始依次取一组：
   - 把组内轮廓拼接成一个完整轮廓。
   - 调用 `DepthGuideLineEstimator::estimate()`。
   - 若返回 `valid = true`，停止尝试。
   - 若失败，继续尝试上一组。

```cpp
const auto groups = findBottomUpMergeGroups(features, valid_indices);

for (size_t g = 0; g < groups.size(); ++g)
{
    const auto& group = groups[g];
    std::vector<cv::Point> merged;
    for (size_t idx : group)
    {
        merged.insert(merged.end(),
                       contours[idx].begin(), contours[idx].end());
    }

    const auto rect_contour = getRectangularContour(merged);
    cv::Mat temp_annotated = annotated.clone();
    GuideLineError candidate = guide_line_estimator_->estimate(
        merged, rect_contour,
        proc_image->size(), rgb_msg->header, temp_annotated);

    if (candidate.valid)
    {
        error_msg = candidate;
        selected_group_idx = static_cast<int>(g);
        annotated = std::move(temp_annotated);
        break;
    }
}
```

### 4.3 判定逻辑

**不主动判断“线是不是在地面上”**，而是让 `DepthGuideLineEstimator` 验证：

- 能反投影出 ≥2 个有效地面点并拟合出直线 → 认为该组合在地面，返回有效结果。
- 所有点反投影失败（`t <= 0`） → 认为该组合不在地面，继续尝试上一组。

### 4.4 可视化

非最终候选组使用临时 `annotated` 图像尝试，避免把失败组的绘制残留在最终发布的标注图上。最终选中组的标注（蓝色轮廓、黄色最小外接矩形）由估计器绘制到正式 `annotated` 图像。

---

## 5. 反投影：像素 → 地面点

源码：`src/depth_guide_line_estimator.cpp` → `pixelToRay()` / `rayPlaneIntersection()`

### 5.1 像素转射线

对缩放后的图像像素 `(u, v)`，使用已缩放的相机内参：

```cpp
ray.x = (u - cx) / fx;
ray.y = (v - cy) / fy;
ray.z = 1.0;
ray.normalize();
```

### 5.2 射线与地平面求交

```cpp
const float denom = normal.dot(ray);
if (std::fabs(denom) < 1e-6f) return false;  // 射线与平面平行

const float t = -d / denom;
if (t <= 0.0f) return false;  // 交点在相机后方或平面上

point = t * ray;
```

### 5.3 有效点筛选

- `skipped_rays`：相机内参无效导致无法生成射线（通常为 0）。
- `skipped_intersections`：射线与平面平行或 `t <= 0`（通常因为点在地平线以上）。
- 仅保留有效地面点用于后续拟合。

---

## 6. 地面直线拟合

源码：`src/depth_guide_line_estimator.cpp` → `fitGroundLine()`

### 6.1 计算地面点中心

```cpp
point = mean(ground_points);
```

### 6.2 协方差矩阵与 PCA

```cpp
cov += (p - point) * (p - point).transpose();
// 取最大特征值对应的特征向量
direction = solver.eigenvectors().col(2);
```

### 6.3 方向对齐到车体前向

引导线是几何无向的。把 PCA 方向对齐到车体前向（相机 Z 轴在地平面的投影；若相机几乎垂直地面则回退到图像“上”方向 `-Y`）：

```cpp
if (direction.dot(forward) < 0.0f)
{
    direction = -direction;
}
```

---

## 7. 误差计算

### 7.1 方向角误差

源码：`src/depth_guide_line_estimator.cpp` → `computeYawError()`

在车体前向 `forward` 和 `left = forward × n` 构成的局部坐标系中：

```cpp
const double angle = std::atan2(ground_direction.dot(left),
                                ground_direction.dot(forward));
```

由于直线无向，同时计算两个相反方向的误差，取绝对值较小者：

```cpp
const double error1 = normalizeAngle(angle - target_angle);
const double error2 = normalizeAngle(angle + M_PI - target_angle);
return (std::abs(error1) < std::abs(error2)) ? error1 : error2;
```

### 7.2 横向误差

源码：`src/depth_guide_line_estimator.cpp` → `computeLateralErrorM()`

1. 计算机光心在地平面的投影：`camera_origin_on_ground = -d * n`。
2. 沿前向 `forward` 前进 `look_ahead_m` 得到前视参考点 `ref_point`。
3. 计算 `ref_point` 到地面直线的有符号距离：

```cpp
const Eigen::Vector3f perp = ground_direction.cross(ground_normal_);
return (ref_point - ground_point).dot(perp);
```

`lateral_error_m > 0` 表示引导线在参考点右侧。

### 7.3 ROI 参考点

为了兼容 `FitLineGuideLineEstimator` 的 `roi_point` 语义，Depth 模式返回前视参考点在归一化图像平面的投影：

```cpp
ref_point = (-d * n) + look_ahead_m * forward;
roi_point.x = ref_point.x / ref_point.z;
roi_point.y = ref_point.y / ref_point.z;
roi_point.z = ref_point.z;
```

---

## 8. 参数说明

参数文件：`config/detector.yaml`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `guide_line_estimator_type` | `DepthGuideLineEstimator` | 必须设为 `DepthGuideLineEstimator` 才启用本文档算法 |
| `target_angle` | `0.0` | 目标引导线方向（rad），相对于车体前向；正常循线设为 `0.0` |
| `look_ahead_m` | `0.0` | 前视参考点距离（m）；`0.0` 表示使用相机光心在地平面的投影 |
| `ground_plane_provider_type` | `topic` | 地平面来源：`topic` / `imu` |
| `ground_plane_topic` | `ground_plane/coefficients` | topic provider 订阅话题 |
| `imu_topic` | `/camera/gyro_accel/sample` | imu provider 订阅话题 |
| `camera_height` | `0.14` | imu provider 相机高度（m） |
| `gravity_filter_alpha` | `1.0` | imu provider 重力低通滤波系数 |
| `imu_to_camera_qx/qy/qz/qw` | `0/0/0/1` | imu provider IMU→相机旋转 |

> 轮廓过滤参数（`min_contour_length`、`min_aspect_ratio`、`enable_contour_merging` 等）在 `color_region_detector` 中处理，详见 `docs/color_region_detector.md`。

---

## 9. 调参指南

### 9.1 地平面来源选择

- 有 `ground_plane_calibrator` 标定的话题 → `ground_plane_provider_type: topic`。
- 只有 IMU 且相机高度固定 → `ground_plane_provider_type: imu`，并准确配置 `camera_height` 和 `imu_to_camera_q`。

### 9.2 target_angle 调节

- 车辆正常循线（引导线在车体前方笔直延伸）→ `target_angle: 0.0`。
- 若引导线在真实世界中有固定偏角（如弯道入口、斜线）→ 按实际角度调整，单位 rad。

### 9.3 look_ahead_m 调节

- `0.0`：横向误差在相机光心正下方地面点计算，对近处横向偏移最敏感。
- 增大到 `0.3~0.5`：在车身前方一段距离处评估横向误差，提前响应弯道，但会引入远处噪声。
- 建议：从 `0.0` 开始，根据实际跟踪效果逐步增加。

### 9.4 地平线以上轮廓误检

如果频繁看到 `not enough ground-plane points: 0 valid`，说明当前尝试的组可能是地平线/背景线：

- 检查 `min_contour_length` 和 `min_aspect_ratio` 是否过滤掉了真正的短线引导线。
- 启用 `enable_contour_merging: true`，把断裂的地面线拼接成更长轮廓，提高排序优先级。
- 自下而上锚点合并会从图像底部开始尝试，通常能优先选中地面线。如果仍频繁失败，说明所有候选组都不在地面，需要检查 HSV 阈值或地平面是否异常。

---

## 10. 常见问题

### 10.1 `not enough ground-plane points: 0 valid ...`

- 地平面符号错误：已自动归一化（见 3.2）。
- 轮廓在地平线以上：当前尝试的组不是地面引导线，自下而上锚点合并会继续尝试上一组。
- 所有组都失败：可能当前帧没有有效地面线，或 HSV 误检太多，需要检查掩码和地平面。
- 相机光轴几乎平行于地面：所有射线 `n · ray ≈ 0`，无法稳定反投影。

### 10.2 `roi_point.z` 接近 0 或极大

- 相机几乎垂直地面（`n` 接近 `-Z`），`ref_point.z` 接近 0，导致投影坐标发散。
- 此时 `forward` 会回退到图像 `-Y` 方向，建议检查相机安装角度。

---

## 11. 关键源码索引

| 文件 | 作用 |
|---|---|
| `include/gemini_geometry_detector/depth_guide_line_estimator.h` | 深度估计器类声明 |
| `src/depth_guide_line_estimator.cpp` | 反投影、PCA、误差计算 |
| `src/color_region_detector.cpp` | 自下而上锚点合并与选择逻辑 |
| `include/gemini_geometry_detector/ground_plane_provider_interface.h` | 地平面来源接口 |
| `src/topic_ground_plane_provider.cpp` | 话题地平面来源 |
| `src/imu_ground_plane_provider.cpp` | IMU 地平面来源 |
| `docs/color_region_detector.md` | 上游轮廓检测与合并逻辑 |

---

## 12. 后续扩展提示

- 需要支持倾斜地面或坡道时，可引入更复杂的地平面跟踪或分段平面估计。
- 需要提高远处引导线精度时，可放宽 `max_contour_points` 或在估计器内部使用全点采样。
- 需要与地图车道线匹配时，可直接使用 `lateral_error_m`（米制）与地图做横向对齐。
