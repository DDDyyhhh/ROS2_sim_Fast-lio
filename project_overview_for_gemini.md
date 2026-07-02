# Orange Pi 5 Plus — Autonomous Lawn Mower Coverage Planning Project

> **给 Gemini 架构师的项目总览文档**
> 目标：在仿真环境中验证自动割草机的全覆盖路径规划（牛耕式），后续部署到实车。

---

## 一、项目背景

本项目在 **Orange Pi 5 Plus**（RK3588, ARM64）单板计算机 + STM32F407 下位机上，搭建一套完整的 **ROS 2 Humble** 自动割草机系统，实现用户在地图上画多边形框定区域 → 牛耕式全覆盖路径规划 → 沿路径执行割草的核心流程。

### 系统架构（三层）

| 层级 | 硬件 | 职责 |
|------|------|------|
| **开发/仿真** | PC (WSL2 Ubuntu 22.04) | 代码编辑 + ROS2 Humble + Ignition Fortress 仿真测试 |
| **上位机** | Orange Pi 5 Plus (RK3588, 16GB) | SLAM建图 (FAST-LIO) + 全覆蓋规划 + Nav2导航 |
| **下位机** | STM32F407 | CAN总线 电机控制 + 刀盘控制 + 安全急停 |

### 传感器

| 传感器 | 型号 | 用途 |
|--------|------|------|
| LiDAR | Mid-360 (固态, non-repetitive scan) | SLAM + 避障 (仿真用32线gpu_lidar) |
| RTK GPS | BT-982G1 (NTRIP 网络RTK) | 绝对定位 + 区域框定 (待硬件到货) |
| IMU | BMI088 (Mid-360内置) | 惯导融合 (仿真200Hz高斯噪声模型) |

---

## 二、当前进度 (2026-07-02)

### ✅ 已完成：Phase 0 — 仿真环境升级

1. **50m×50m 室外草坪仿真世界** (`worlds/grassland_50x50.world`)
   - 绿色草地（box 50×50×0.1）+ 方向阳光 + 环境光
   - 5棵树木（圆柱树干 + 球体树冠）
   - 2个花坛（box）、3块石头（sphere）、3个喷灌头（小圆柱）
   - 边界标记（黄色线条，仅视觉）
   - 开放室外（无墙无天花板）
   - 物理引擎 200Hz，ogre 渲染（ARM兼容）
2. **随机障碍物生成脚本** (`scripts/random_obstacles.py`)
   - 4种模板（树/花坛/石头/喷灌头），参数化几何和颜色
   - `--count 10 --seed 42 --append` 追加到世界文件
   - 防重叠放置
3. **Nav2 双模式参数文件**
   - `config/nav2_params.yaml` — 纯导航模式（AMCL + 预建地图）
   - `config/nav2_slam_params.yaml` — SLAM+导航模式（FAST-LIO 实时建图）

### ✅ 已完成：Phase 1 — 全覆盖路径规划（核心功能）

新建了 `mower_coverage` Python 包，包含以下模块：

| 模块 | 文件 | 功能 |
|------|------|------|
| **牛耕式路径规划器** | `mower_coverage/boustrophedon_planner.py` | 核心算法：多边形区域 → 平行条带 → 障碍物跳过 → 转弯连接 |
| **区域定义** | `mower_coverage/area_definer.py` | 服务接口 `/coverage/set_area`、YAML持久化、RViz可视化 |
| **路径执行器** | `mower_coverage/path_executor.py` | direct/Nav2双模式、断点续割、边界保护、刀盘控制 |
| **覆盖率监控** | `mower_coverage/coverage_monitor.py` | 0.2m栅格热力图、已覆盖m²实时统计 |
| **整合演示** | `mower_coverage/coverage_demo.py` | 一键 `/coverage/plan` 出50m×50m路径 |
| **启动文件** | `launch/coverage_planning.launch.py` | 同时启动4个节点 |
| **配置文件** | `config/coverage_params.yaml` | 所有可调参数（割幅/重叠/速度） |
| **单元测试** | `tests/test_planner.py` | 5/5 测试通过 |

**算法验证结果：**
```
50m × 50m 区域, 0.5m 割幅, 10% 重叠
→ 条带间距 0.45m
→ 生成 18885 个路径点
→ 总路径长度 7285.7m
→ 5个测试全部通过 ✅
```

### ⏳ 待完成（硬件到货后）

| 阶段 | 任务 | 依赖 |
|------|------|------|
| **Phase 2** | RTK GPS + LiDAR 融合（BT-982G1 NTRIP + nmea_navsat_driver + robot_localization EKF） | Mid-360, BT-982G1 |
| **Phase 2** | Mid-360 LiDAR 驱动（Livox SDK2 + FAST-LIO 适配） | Mid-360 |
| **Phase 3** | CAN 总线协议设计与 STM32F407 固件 | 下位机电路 |
| **Phase 3** | 安全机制（电子围栏/碰撞检测/倾覆保护） | 下位机 |
| **Phase 4** | 室外实车测试（仿真 → 半实物 → 小范围 → 全尺寸） | 整车 |

---

## 三、技术栈

| 层次 | 技术选型 | 版本 |
|------|---------|------|
| **操作系统** | Ubuntu | 22.04 LTS (Jammy) |
| **中间件** | ROS 2 | Humble Hawksbill |
| **DDS** | Eclipse CycloneDDS（实车）；默认FastRTPS（WSL仿真） | — |
| **仿真引擎** | Ignition Fortress (Gazebo Sim) | 6.x, SDF 1.8 |
| **仿真桥接** | ros_gz_bridge / ros_gz_sim | Humble 官方 |
| **SLAM 建图** | FAST-LIO (FAST_LIO_ROS2) | 自编译 |
| **导航堆栈** | Nav2 (nav2_bringup) | Humble 官方 |
| **全覆盖路径** | Boustrophedon Decomposition（自行实现） | shapely + numpy |
| **GPS融合** | robot_localization（navsat_transform + EKF） | — |
| **下位机通信** | CAN 总线 (socketcan) | — |
| **可视化** | RViz2 + 覆盖率热力图 | Humble 官方 |

---

## 四、系统架构

### 4.1 数据流架构（含覆盖规划）

```
                 上位机 (PC / RK3588)
        ┌─────────────────────────────────────────────────────┐
        │                    ROS 2 中间件                       │
        │                                                      │
        │  ┌──────────┐   ┌─────────────────┐   ┌──────────┐  │
        │  │ 区域定义  │   │ 牛耕式路径规划器  │   │ 路径执行器 │  │
        │  │ area_    │──→│ boustrophedon_   │──→│ path_     │  │
        │  │ definer  │   │ planner/cov_demo │   │ executor  │  │
        │  └──────────┘   └─────────────────┘   └─────┬─────┘  │
        │                                              │        │
        │  ┌──────────────────────────────────────┐    │        │
        │  │      Nav2 导航堆栈                    │    │        │
        │  │  ┌────────┐  ┌──────────┐  ┌──────┐ │    │        │
        │  │  │AMCL/   │  │ NavFn    │  │ DWB  │ │    │        │
        │  │  │FAST-LIO│→│ 全局规划  │→│局部规划│ │    │        │
        │  │  │定位    │  │          │  │      │ │    │        │
        │  │  └────────┘  └──────────┘  └──────┘ │    │        │
        │  └──────────────────────────────────────┘    │        │
        │                                              │        │
        │  ┌─────────────────┐   ┌─────────────────┐   │        │
        │  │  覆盖率监控     │   │ FAST-LIO SLAM   │   │        │
        │  │  coverage_      │   │ (LiDAR+IMU)     │   │        │
        │  │  monitor        │   │ map←odom TF     │   │        │
        │  └────────┬────────┘   └─────────────────┘   │        │
        └───────────┼───────────────────────────────────┘        │
                    │ /cmd_vel                          │        │
                    ▼                                  │        │
        ┌───────────────────────────────────────────┐  │        │
        │       CAN 总线 (socketcan)                 │  │        │
        └────────────────┬──────────────────────────┘  │        │
                         │                             │        │
        ┌────────────────▼──────────────────────────┐  │        │
        │          STM32F407 下位机                  │  │        │
        │  ┌─────────┐ ┌──────────┐ ┌───────────┐  │  │        │
        │  │ 电机控制 │ │ 刀盘控制 │ │ 安全监控   │  │  │        │
        │  │ (PID)   │ │ (启/停)  │ │ (碰撞/倾覆)│  │  │        │
        │  └─────────┘ └──────────┘ └───────────┘  │  │        │
        └───────────────────────────────────────────┘  │        │
```

### 4.2 全覆蓋路径规划流程

```
用户操作：
  在地图上画多边形框定割草区域
        │
        ▼
  区域定义 (area_definer)
  · 保存为 ~/mowing_area.yaml (WGS84 或 局部坐标)
  · 支持内岛障碍物（花坛/树木）
        │
        ▼
  牛耕式路径规划 (boustrophedon_planner)
  · 输入：多边形 + 割幅 + 重叠率 + 起始点
  · 算法：
    1. 沿区域长轴旋转对齐
    2. 生成平行等距条带（spacing = 割幅 × (1-重叠率)）
    3. 对每个条带做多边形裁剪
    4. 移除与障碍物相交的段
    5. 交替方向连接（zigzag）
    6. 添加U形转弯连接点
  · 输出：/coverage_path (nav_msgs/Path, 0.3m间距)
        │
        ▼
  路径执行 (path_executor)
  · direct 模式：逐点导航 + PID-like 控制 → /cmd_vel
  · nav2 模式：调用 Nav2 NavigateToPose action
  · 遇到障碍物（人）：暂停刀盘 → 等待 → 断点续割
  · 断点保存到 ~/coverage_checkpoint.json
        │
        ▼
  覆盖率监控 (coverage_monitor)
  · 0.2m 分辨率栅格图
  · 实时显示已覆盖/总面积 m²
  · 热力图 RViz marker
```

### 4.3 TF 树（两种模式同 Phase 0）

**模式 A — 纯导航（AMCL + 预建地图）：**
```
map
  └── odom (由 AMCL 粒子滤波发布)
        └── body (由 robot_state_publisher /tf_static 发布)
              ├── lidar_link   (fixed, z=0.1)
              ├── imu_link     (fixed, z=0.1)
              ├── left_front_wheel_joint
              ├── left_rear_wheel_joint
              ├── right_front_wheel_joint
              └── right_rear_wheel_joint
```

**模式 B — SLAM+导航（FAST-LIO 实时里程计）：**
```
map ──(static)──→ odom ──(static)──→ camera_init ──(FAST-LIO)──→ body
```

### 4.4 机器人参数

| 参数 | 值 |
|------|-----|
| 底盘尺寸 | 0.4m × 0.3m × 0.15m |
| 底盘质量 | 10.0 kg |
| 轮距 | 0.58 m |
| 轮半径 | 0.08 m |
| 驱动方式 | 四轮差速滑移转向 (skid-steer) |
| 机器人坐标系 frame | `body` |
| **割幅** | **0.5 m (40-60cm, 取中值)** |
| **条带重叠率** | **10%** |
| **有效条带间距** | **0.45 m** |
| **导航速度** | **0.5 m/s** |

---

## 五、全覆盖路径规划算法详解

### 5.1 BoustrophedonPlanner 类

```python
class BoustrophedonPlanner:
    def __init__(self, cutting_width=0.5, overlap=0.1, angle=0.0,
                 start_from_corner=True):
        # 条带间距 = 0.5 * 0.9 = 0.45m
        self.swath_spacing = cutting_width * (1.0 - overlap)

    def plan(self, area_polygon: Polygon,
             obstacles: List[Polygon] = None,
             start_pose: Tuple[float,float,float] = None) -> List[Tuple]:
        """返回有序路径点列表 [(x,y,yaw), ...]
        - 0.3m 间距平滑路径
        - 交替方向 zigzag
        - 障碍物剪裁
        """

    def set_angle(self, angle_deg: float):
        """旋转路径方向（用于手动调整）"""
```

### 5.2 障碍物处理策略

当前实现：**Skip-and-turn**（跳过障碍物覆盖的条带段）
- 条带与障碍物多边形求交集 → 移除相交段
- 障碍物前后保留路径点 → 小车走直线绕过

未来优化：**Boustrophedon Decomposition with exact cell decomposition**
- 按障碍物边界将区域分割为子多边形
- 每个子多边形独立牛耕
- 子多边形间用最短路径连接

### 5.3 动态障碍物处理

- 人（行人）进入工作区 → path_executor 暂停刀盘 → 等待人离开 → **断点续割**
- 断点记录在 `~/coverage_checkpoint.json`（已完成路径索引 + 位置）
- 避免 Nav2 默认"绕过去"导致的漏割

---

## 六、仿真世界详情

### 6.1 grassland_50x50.world

| 元素 | 数量 | 描述 |
|------|------|------|
| 草地 | 1 | 50×50×0.1m 绿色 box |
| 阳光 | 1 | `<directional>` 白色光 + 环境光补光 |
| 树木 | 5 | 0.15m半径圆柱树干 + 1.5m半径球体树冠 |
| 花坛 | 2 | 2×1×0.3m 棕色 box |
| 石头 | 3 | 0.3-0.5m半径 sphere |
| 喷灌头 | 3 | 0.05m半径 × 0.3m高 红色圆柱 |
| 边界标记 | 4 | 黄色细长方体（仅视觉，不参与碰撞） |
| 渲染引擎 | — | `ogre`（非 ogre2，ARM64 兼容） |
| 物理步长 | — | 200 Hz (dt=0.005) |

### 6.2 随机障碍物生成

```bash
python3 scripts/random_obstacles.py --count 15 --seed 42 --append
```
- 4 种模板：树(树冠+树干)、花坛、石头、喷灌头
- 自动检测碰撞，确保不重叠
- `--append` 追加到 .world 文件

---

## 七、mower_coverage 包详解

### 7.1 节点接口

#### area_definer
| 服务 | 类型 | 说明 |
|------|------|------|
| `/coverage/set_area` | `MowerSrv` (自定义) | 设置多边形顶点 |
| `/coverage/save_area` | `std_srvs/Trigger` | 保存到YAML |
| `/coverage/load_area` | `std_srvs/Trigger` | 加载YAML |
| `/coverage/clear_area` | `std_srvs/Trigger` | 清除区域 |

| 发布话题 | 类型 | 说明 |
|---------|------|------|
| `/coverage/area_marker` | `visualization_msgs/Marker` | 多边形边界线 |
| `/coverage/area_fill` | `visualization_msgs/Marker` | 填充面 |
| `/coverage/area_vertices` | `visualization_msgs/MarkerArray` | 顶点球体 |

#### coverage_demo
| 服务 | 说明 |
|------|------|
| `/coverage/plan` | 规划全覆盖路径 |
| `/coverage/plan_and_start` | 规划并立即开始执行 |

| 发布话题 | 说明 |
|---------|------|
| `/coverage/path` | `nav_msgs/Path` 规划结果 |
| `/coverage/path_marker` | RViz LineStrip + 箭头 |

#### path_executor
| 服务 | 说明 |
|------|------|
| `/coverage/start` | 开始执行路径 |
| `/coverage/pause` | 暂停（刀盘停止） |
| `/coverage/resume` | 恢复（断点续割） |
| `/coverage/stop` | 停止并回泊 |

| 发布话题 | 说明 |
|---------|------|
| `/cmd_vel` | 速度控制 |
| `/blade_enabled` | 刀盘启停 |

#### coverage_monitor
| 发布话题 | 说明 |
|---------|------|
| `/coverage/grid` | `visualization_msgs/MarkerArray` 栅格热力图 |
| `/coverage/coverage_path_line` | 已覆盖路径轨迹 |
| `/coverage/stats` | 覆盖统计数据 |

### 7.2 启动命令

```bash
# 终端1: 仿真
ros2 launch outdoor_sim sim_launch.py

# 终端2: 覆盖规划系统
ros2 launch mower_coverage coverage_planning.launch.py

# 终端3: 规划并开始
ros2 service call /coverage/plan_and_start std_srvs/srv/Trigger
```

### 7.3 服务调用

```bash
# 手动规划（使用默认50m×50m区域）
ros2 service call /coverage/plan std_srvs/srv/Trigger

# 开始执行
ros2 service call /coverage/start std_srvs/srv/Trigger

# 暂停
ros2 service call /coverage/pause std_srvs/srv/Trigger

# 断点恢复
ros2 service call /coverage/resume std_srvs/srv/Trigger

# 停止并返回
ros2 service call /coverage/stop std_srvs/srv/Trigger

# 查看覆盖率统计
ros2 topic echo /coverage/stats
```

---

## 八、当前项目文件结构

```
~/ros2_ws/
├── src/
│   ├── outdoor_sim/                          # 仿真包
│   │   ├── worlds/
│   │   │   ├── grass_terrain.world           # 旧室内10m×10m（已弃用）
│   │   │   └── grassland_50x50.world         # ★ 新 50m×50m 室外草坪
│   │   ├── urdf/
│   │   │   └── robot_sensors.xacro           # 4WD 机器人 (32线 LiDAR + 200Hz IMU)
│   │   ├── launch/
│   │   │   ├── sim_launch.py                 # 仿真启动
│   │   │   ├── nav2_sim_launch.py            # 模式 A: 纯导航
│   │   │   └── slam_nav2_launch.py           # 模式 B: SLAM+导航
│   │   ├── config/
│   │   │   ├── nav2_params.yaml              # Nav2 纯导航参数
│   │   │   ├── nav2_slam_params.yaml         # Nav2 SLAM+导航参数
│   │   │   └── nav2_3d_view.rviz             # Nav2 RViz 配置
│   │   ├── scripts/
│   │   │   ├── odom_to_tf.py                 # /odom→/tf 中继
│   │   │   ├── clock_filter.py               # 仿真时钟滤波器
│   │   │   └── random_obstacles.py           # ★ 随机障碍物生成器
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── mower_coverage/                       # ★ 全覆盖路径规划包（新）
│   │   ├── mower_coverage/
│   │   │   ├── __init__.py
│   │   │   ├── boustrophedon_planner.py      # 核心牛耕式算法
│   │   │   ├── area_definer.py               # 区域定义节点
│   │   │   ├── path_executor.py              # 路径执行器
│   │   │   ├── coverage_monitor.py           # 覆盖率监控
│   │   │   └── coverage_demo.py              # 整合演示
│   │   ├── launch/
│   │   │   └── coverage_planning.launch.py   # 启动文件
│   │   ├── config/
│   │   │   └── coverage_params.yaml          # 可调参数
│   │   ├── tests/
│   │   │   └── test_planner.py              # 5个单元测试
│   │   ├── setup.py
│   │   └── package.xml
│   │
│   └── FAST_LIO_ROS2/                        # SLAM 建图包
│       ├── config/
│       │   └── mid360.yaml                   # FAST-LIO 仿真配置
│       ├── launch/
│       │   └── mapping.launch.py
│       ├── src/
│       ├── include/
│       └── package.xml
│
├── pcd_to_pgm.py                             # PCD → PGM 转换
├── save_map.py                               # 地图保存工具
├── CLAUDE.md                                 # 项目规则
└── project_overview_for_gemini.md            ← 本文件
```

---

## 九、导航/规划参数摘要

### 9.1 Nav2 全局规划

| 参数 | 模式 A | 模式 B |
|------|--------|--------|
| 全局代价地图 | 60×60m, 0.1m/px | 100×100m, 0.1m/px |
| 局部代价地图 | 6×6m rolling | 6×6m rolling |
| 规划器 | Navfn (A*) | Navfn (A*) |
| 控制器 | DWB | DWB (允许倒车) |
| 最大速度 | 0.5 m/s | 0.5 m/s |
| 膨胀半径 | 0.4-0.6m | 0.4-0.6m |

### 9.2 覆盖规划参数

| 参数 | 默认值 | 范围 |
|------|--------|------|
| 割幅 | 0.5 m | 0.4-0.6 m |
| 重叠率 | 10% | 5-20% |
| 条带间距 | 0.45 m | 0.32-0.57 m |
| 路径间距 | 0.3 m | — |
| 执行速度 | 0.5 m/s | 0.2-0.8 m/s |

### 9.3 FAST-LIO (仿真)

| 参数 | 值 |
|------|-----|
| lidar_type | 2 (PointCloud2) |
| scan_line | 32 |
| point_filter_num | 3 (降采样) |
| filter_size_surf | 0.15 |
| filter_size_map | 0.15 |
| extrinsic_T | [0, 0, 0.03] |
| max_iteration | 3 |

---

## 十、已知问题与风险

| # | 问题 | 状态 | 说明 |
|---|------|------|------|
| 1 | FAST-LIO 无回环检测 | ⚠️ 已知 | 长时间运行地图会漂移，需 GPS 辅助 |
| 2 | PCD→PGM 质量 | ⚠️ 待验证 | 需实际建图后验证2D投影 |
| 3 | **模式 A/B 端到端仿真** | 🔴 待验证 | 参数已调优，需在 50m 大草坪实测 |
| 4 | 空旷区域 FAST-LIO 特征不足 | 🔴 已知 | 可能需要 GPS 辅助初始化 |
| 5 | 室外光线对 LiDAR 干扰 | ⚠️ 待实车 | 阳光直射可能产生噪点 |
| 6 | **全覆盖路径规划仿真验证** | 🔴 待进行 | 启动全部节点，观察覆盖率可达 ~96%+ |
| 7 | 动态障碍物（路人）暂停恢复 | ⚠️ 已实现待测试 | path_executor 断点逻辑需仿真验证 |
| 8 | RTK GPS 融合未集成 | 🔴 Phase 2 | 等待 BT-982G1 硬件到货 |
| 9 | CAN 总线协议未设计 | 🔴 Phase 3 | 等待 STM32 下位机 |
| 10 | 覆盖率验证工具 | ✅ 已实现 | coverage_monitor 输出热力图 + 统计数据 |
| 11 | 断点续割 | ✅ 已实现 | checkpoint JSON 持久化 |
| 12 | 区域定义持久化 | ✅ 已实现 | YAML 序列化/反序列化 |

---

## 十一、启动命令速查

### 仿真 + 覆盖规划

```bash
# ── 环境变量（WSL 使用默认 DDS，实车用 CycloneDDS） ──
source install/setup.bash

# ── 1. 启动 50m×50m 草坪仿真 ──
sim_launch.py 需修改 world 路径指向 grassland_50x50.world

# ── 2. 启动全覆盖路径规划系统 ──
ros2 launch mower_coverage coverage_planning.launch.py

# ── 3. 一键规划+执行 ──
ros2 service call /coverage/plan_and_start std_srvs/srv/Trigger

# ── 4. 使用自定义区域 ──
# 用 RViz 的 "Publish Point" 工具采集多边形顶点
# 通过服务设置区域
ros2 service call /coverage/set_area ...

# ── 5. 监控覆盖率 ──
ros2 topic echo /coverage/stats
```

### 导航（独立使用）

```bash
# 模式 A：纯导航（预建地图 + AMCL）
ros2 launch outdoor_sim nav2_sim_launch.py

# 模式 B：SLAM+导航（未知环境探索）
ros2 launch outdoor_sim slam_nav2_launch.py
```

### 建图

```bash
ros2 launch fast_lio mapping.launch.py config:=mid360.yaml
```

---

## 十二、关键未解决问题

> **给 Gemini 架构师的核心问题：**

1. **全覆盖路径在50m草坪能否跑通？** — 牛耕式算法已测试通过，但 RViz 端到端可视化验证尚未执行：`coverage_demo` 发布 /coverage/path 后，小车能否正确跟踪？

2. **RTK 融合方案** — `robot_localization` 的 `navsat_transform_node` 需要 GPS 先收敛到固定解再启动 EKF。BT-982G1 NTRIP 在空旷草地收敛速度如何？

3. **空旷环境 FAST-LIO 退化** — 50m × 50m 仅有少量树干特征，是否会导致 FAST-LIO 里程计发散？是否需要加入 GPS 辅助的松耦合模式？

4. **滑移转向对覆盖率影响** — 差速滑移在草坪上转弯时会破坏草皮 + 实际路径与规划路径偏差增大，覆盖率容差需要实测调整。

5. **行人检测** — 是否需要在 Mid-360 点云上加上行人识别算法，还是仅依赖 Nav2 的局部代价地图避障？

---

> **版本记录：** 2026-07-02 — Phase 0+1 完成。下一阶段 Phase 2 等待 Mid-360 和 BT-982G1 硬件到货后开始。
