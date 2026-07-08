# 斜坡地形 + 多区域全覆盖路径规划 — 设计文档

> 日期：2026-07-03
> 状态：已批准
> 对应计划：`/home/yh/.claude/plans/codex-claude-dapper-quasar.md`

## 概述

在现有 Phase 0+1 仿真基础上（50×50m 平面草坪 + 单区域牛耕式规划），新增：

1. **30×30m 起伏地形仿真世界** — 含 Mid-360 模拟 LiDAR + RTK GPS 传感器
2. **多区域框定系统** — 支持多个不相连草地区域，每个支持内岛（禁入区）
3. **多区域牛耕式规划与执行** — 逐区域规划，含区域间导航路径

**核心约束**：不修改任何现有文件，全部新增独立模块。

## 文件清单（12 个新文件）

### outdoor_sim 包（6 个）

| # | 文件 | 说明 |
|---|------|------|
| 1 | `scripts/generate_heightmap.py` | 生成高度图 PNG |
| 2 | `worlds/heightmap_30x30.png` | 512×512 灰度高度图 |
| 3 | `worlds/hill_terrain_30x30.world` | 新 SDF 世界（heightmap 地形 + 障碍物） |
| 4 | `urdf/robot_with_gps.urdf` | 带 Mid-360 + NavSat 的小车 URDF |
| 5 | `config/hill_sensor_params.yaml` | LiDAR/GPS 传感器参数 |
| 6 | `launch/hill_sim_launch.py` | 新仿真启动文件 |

### mower_coverage 包（6 个）

| # | 文件 | 说明 |
|---|------|------|
| 7 | `mower_coverage/multi_area_definer.py` | 多区域定义节点 |
| 8 | `config/areas_example.yaml` | 预设区域（含内岛） |
| 9 | `config/hill_coverage_params.yaml` | 斜坡场景参数 |
| 10 | `mower_coverage/hill_boustrophedon.py` | 增强牛耕式规划器 |
| 11 | `mower_coverage/multi_area_executor.py` | 多区域执行器 |
| 12 | `launch/hill_coverage.launch.py` | 新启动文件 |

## 地形设计

- 30m × 30m, 高度范围 0–4m
- Perlin 噪声 + 人工特征生成的 512×512 灰度图
- **西北丘**：凸起 2.5m
- **东南洼地**：凹陷 1.5m（池塘位置）
- **中部缓坡**：±0.5m
- **东北–西南脊线**：连贯脊线

### 障碍物布局

- 池塘（东南洼地）：蓝色半透明圆柱，碰撞+视觉
- 陡坡禁入区（西北丘顶）：红色标记
- 灌木丛 ×3（脊线两侧）
- 树 ×4 + 石头 ×2（散落）

## 传感器仿真

- **Mid-360 LiDAR**：6 线，1200 采样/圈，FOV -7.4°~+52.2°，20Hz
- **RTK GPS**：NavSat 传感器，2cm 噪声，10Hz，深圳基准坐标
- **EKF 融合**：robot_localization 的 navsat_transform + ekf_localization

## 多区域系统

- YAML 定义 + RViz PublishPoint 两种输入方式
- 每个区域支持外边界 + 内岛（禁入区）
- 8 个服务接口（add_point, finish_polygon, save, load 等）
- 彩色多边形可视化

## 规划与执行

- 逐区域牛耕式规划（2D 投影）
- 区域间直线导航路径
- 地形感知速度调整（坡度 > 20° 降速 50%）
- 按区域 checkpoint 断点续割

## 使用方式

```bash
# 启动仿真
ros2 launch outdoor_sim hill_sim_launch.py

# 启动覆盖规划
ros2 launch mower_coverage hill_coverage.launch.py

# 加载预设区域
ros2 service call /multi_area/load std_srvs/srv/Trigger "{}"

# 开始规划与执行
ros2 service call /multi_area/plan_and_start std_srvs/srv/Trigger "{}"
```

## 与 Phase 2 的关系

本设计完全独立于现有文件，完成后可无缝继续 Phase 2（实车部署）。斜坡世界的增强传感器仿真为实车 EKF 融合提供预测试环境。
