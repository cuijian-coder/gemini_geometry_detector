# RectangleGuideLineEstimator 说明

`RectangleGuideLineEstimator` 是一种基于**地面矩形带（OBB，Oriented Bounding Box）中心轴**的引导线估计器。

它继承自 `IGuideLineEstimator`，与 `DepthGuideLineEstimator` 一样需要地面平面（topic 或 IMU 提供），但用矩形拟合代替原来的 PCA 直线拟合，更适合**远端轮廓抖动、不规则、断裂**的场景。

---

## 1. 适用场景

- 引导线宽度固定（如 10 cm 地面胶带）。
- RGB 远端提取的轮廓抖动、毛刺多，PCA 主方向不稳定。
- 实际运行中轮廓可能出现断裂或不规则。

---

## 2. 算法流程

```text
轮廓像素
   ↓ 反投影射线
地面平面交点 → 地面 3D 点云
   ↓ PCA 求主方向
长轴方向 = 引导线方向
短轴宽度 ≈ 引导线物理宽度
   ↓ 取矩形中心轴
OBB 中心 = 参考点
   ↓ 对齐车辆前向
消除 180° 方向歧义
   ↓ 计算控制量
yaw_error / lateral_error_m
```

### 2.1 反投影

与 `DepthGuideLineEstimator` 相同：

```text
ray = ((u - cx) / fx, (v - cy) / fy, 1)
t = -d / (normal · ray)
P_ground = t * ray
```

只保留 `t > 0`（相机前方）的点。

### 2.2 OBB 拟合

1. 计算地面点云的质心 `c`。
2. 计算 3x3 协方差矩阵。
3. Eigen 分解：
   - `col(2)`：最大特征值方向 → 矩形**长轴**
   - `col(1)`：中间特征值方向 → 矩形**短轴**
   - `col(0)`：最小特征值方向 → 近似地面法向
4. 将点投影到长轴/短轴，得到矩形范围。
5. 矩形中心 = 质心，长轴方向 = 引导线方向。

### 2.3 宽度校验（软校验）

- 实际短轴宽度与期望宽度 `guide_line_width_m` 比较。
- 偏差超过 `obb_width_tolerance_m` 时打印 WARN，但**仍然返回结果**。
- 同时统计超出期望带（`guide_line_width_m / 2 + tolerance`）的离群点比例。
- 离群比例超过 `obb_max_outlier_ratio` 时打印 WARN，同样不硬拒绝。

### 2.4 方向对齐

长轴方向与车辆前向（相机 Z 在地平面投影）对齐，解决 180° 方向歧义。

### 2.5 误差计算

- `yaw_error`：长轴方向与 `target_angle` 的偏差。
- `lateral_error_m`：前视参考点到中心轴的有向距离。
  - 参考点 = 相机光心在地平面投影 + `look_ahead_m * forward`
  - 使用 OBB 中心作为线上参考点。

---

## 3. 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `guide_line_estimator_type` | `"DepthGuideLineEstimator"` | 设为 `"RectangleGuideLineEstimator"` 启用 |
| `guide_line_width_m` | 0.10 | 引导线物理宽度（m） |
| `obb_width_tolerance_m` | 0.05 | 宽度偏差告警阈值（m） |
| `obb_max_outlier_ratio` | 0.30 | 超出期望带的点比例告警阈值 |
| `obb_min_ground_points` | 10 | 最少有效地面点数，低于则失败 |
| `look_ahead_m` | 0.0 | 前视参考点距离（m） |

---

## 4. 与 DepthGuideLineEstimator 对比

| 特性 | DepthGuideLineEstimator | RectangleGuideLineEstimator |
|---|---|---|
| 拟合模型 | PCA 直线 | OBB 矩形中心轴 |
| 对远端抖动 | 全部点直接做主方向 | 矩形边界抑制不规则点 |
| 对线宽信息 | 不利用 | 利用并软校验 |
| 断裂线 | 受断点分布影响 | 更稳定 |
| 计算量 | 较小 | 略大 |

---

## 5. 调参建议

- 引导线宽度不准 → 调整 `guide_line_width_m`。
- 远端抖动导致宽度告警频繁 → 增大 `obb_width_tolerance_m`。
- 噪声点太多 → 减小 `obb_max_outlier_ratio` 或配合 `mask_filter_alpha`。
- 地面点太少失败 → 减小 `obb_min_ground_points`（但不建议低于 6）。
- 效果仍不理想 → 可切换回 `DepthGuideLineEstimator` 对比。

---

## 6. 源码索引

| 文件 | 作用 |
|---|---|
| `include/gemini_geometry_detector/rectangle_guide_line_estimator.h` | 类声明 |
| `src/rectangle_guide_line_estimator.cpp` | OBB 拟合、反投影、误差计算 |
| `include/gemini_geometry_detector/guide_line_estimator_interface.h` | 接口定义 |
| `src/color_region_detector.cpp` | 工厂创建、参数加载 |
| `config/detector.yaml` | 运行参数 |
