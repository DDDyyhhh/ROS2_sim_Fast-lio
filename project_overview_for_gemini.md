# Orange Pi 5 Plus — Autonomous 4WD LiDAR SLAM & Navigation Project

> **给 Gemini 架构师的项目总览文档**
> 目标：在仿真环境中为四驱差速小车接入 Nav2 导航堆栈，实现自主避障与路径规划。

---

## 一、项目背景

本项目在 **Orange Pi 5 Plus**（RK3588, ARM64）单板计算机上，搭建了一套完整的 **ROS 2 Humble** + **Ignition Fortress** 仿真系统，模拟一台装备 **DJI Mid-360 等效 LiDAR** + **200Hz IMU** 的四驱差速小车，在封闭室内环境中进行 **FAST-LIO SLAM** 建图。

当前已实现：小车底盘物理仿真、LiDAR/IMU 传感器仿真、ros_gz_bridge 话题桥接、FAST-LIO 实时建图、关节状态发布。**下一步目标是接入 Nav2 导航堆栈，实现自主避障与路径规划。**

---

## 二、硬件平台

| 项目 | 规格 |
|------|------|
| **主板** | Orange Pi 5 Plus |
| **SoC** | Rockchip RK3588 (8核: 4×Cortex-A76 + 4×Cortex-A55) |
| **RAM** | 16 GB |
| **GPU** | Mali-G610 MP4 (**已禁用硬件加速**，使用软件渲染) |
| **存储** | 板载 eMMC / microSD |
| **OS** | Ubuntu 22.04 LTS (Jammy Jellyfish) |
| **架构** | aarch64 (ARM64) |

### RK3588 已知硬伤

| 问题 | 影响 | 对策 |
|------|------|------|
| Mali-G610 GPU 驱动不稳定 | Ogre2 渲染 segfault (exit -11) | `LIBGL_ALWAYS_SOFTWARE=1` 强制软件渲染 |
| 8 核 ARM 多线程竞争 | colcon 编译锁死系统 | `export MAKEFLAGS="-j8"` 硬限 8 线程 |
| 有限的内存带宽 | 高频率传感器 RELIABLE QoS 导致 DDS 溢出 | LiDAR/IMU 强制 `BEST_EFFORT` QoS |
| Heightmap shader 不兼容 | `<heightmap>` 导致 Ogre shader 崩溃 | 禁止使用 heightmap，改用 `<box>`  |

---

## 三、软件技术栈

| 层次 | 技术选型 | 版本 |
|------|---------|------|
| **操作系统** | Ubuntu | 22.04 LTS (Jammy) |
| **中间件** | ROS 2 | Humble Hawksbill |
| **DDS** | Eclipse CycloneDDS | (默认 FastRTPS 不兼容 RK3588) |
| **仿真引擎** | Ignition Fortress (Gazebo Sim) | 6.x, SDF 1.8 |
| **仿真桥接** | ros_gz_bridge / ros_gz_sim | (Humble 官方) |
| **SLAM 建图** | FAST-LIO (FAST_LIO_ROS2) | 自编译 aarch64 |
| **URDF 解析** | robot_state_publisher + xacro | Humble 官方 |
| **可视化** | RViz2 | Humble 官方 |
| **C++ 标准** | C++17, aarch64 NEON 优化 | `-march=armv8-a+crypto -mcpu=cortex-a76` |

---

## 四、系统架构

### 4.1 数据流架构

```
┌─────────────────────────────────────────────────────────────────────────┐
│                    Ignition Fortress 仿真世界                              │
│                                                                          │
│  ┌──────────────────────────────────┐                                    │
│  │   World: outdoor_flat_features    │  (10m×10m 封闭房间 + 3 柱子障碍物)   │
│  │   · 地面 / 四面围墙 / 天花板       │                                    │
│  │   · 3 个 box 柱子障碍物            │                                    │
│  └──────────────┬───────────────────┘                                    │
│                 │                                                        │
│  ┌──────────────▼───────────────────┐                                    │
│  │   Robot: outdoor_bot (4WD)       │                                    │
│  │  ┌────────────────────────────┐  │                                    │
│  │  │ DiffDrivePlugin (skid)     │  │  ← /model/outdoor_bot/cmd_vel     │
│  │  │  4× continuous joints      │  │                                    │
│  │  │  wheel_sep: 0.36m          │  │                                    │
│  │  │  wheel_radius: 0.08m       │  │                                    │
│  │  │  odom → /odom @20Hz        │  │                                    │
│  │  └────────────────────────────┘  │                                    │
│  │  ┌────────────────────────────┐  │                                    │
│  │  │ LiDAR (gpu_lidar→cpu ray)  │  │  → /lidar/points (10Hz)           │
│  │  │  32线, 360°H, 59°V         │  │                                    │
│  │  │  range: 0.1-40m           │  │                                    │
│  │  └────────────────────────────┘  │                                    │
│  │  ┌────────────────────────────┐  │                                    │
│  │  │ IMU (200Hz, 6-axis noise)  │  │  → /imu/data (200Hz)              │
│  │  └────────────────────────────┘  │                                    │
│  │  ┌────────────────────────────┐  │                                    │
│  │  │ JointStatePublisher(30Hz)  │  │  → /joint_state                    │
│  │  └────────────────────────────┘  │                                    │
│  └──────────────────────────────────┘                                    │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │
                               │ ros_gz_bridge parameter_bridge
                               ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                           ROS 2 层                                        │
│                                                                           │
│  /clock              →  rosgraph_msgs/Clock                               │
│  /lidar/points       →  sensor_msgs/PointCloud2  (remap → /velodyne_points) │
│  /imu/data           →  sensor_msgs/Imu                                   │
│  /model/outdoor_bot/ →  model/outdoor_bot/joint_state                      │
│   joint_state             sensor_msgs/JointState   (remap → /joint_states)  │
│                                                                           │
│  /cmd_vel (ROS)      →  /model/outdoor_bot/cmd_vel (Ignition) ← 控制命令   │
│                                                                           │
│  ┌──────────────────────────────────────────────────────────────────┐    │
│  │                   FAST-LIO SLAM 节点                              │    │
│  │  /velodyne_points + /imu/data → odometry → /Odometry            │    │
│  │                                     TF: map ← odom               │    │
│  │                                     → /fastlio_mapping/global_map │    │
│  └──────────────────────────────────────────────────────────────────┘    │
│                                                                           │
│  ┌──────────────────────────────────────────────────────────────────┐    │
│  │              robot_state_publisher                               │    │
│  │  URDF/xacro → /tf_static (body, lidar_link, imu_link, 4 wheels)  │    │
│  └──────────────────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────────────────┘
```

### 4.2 TF 树

```
map → odom (FAST-LIO 发布)
  └── base_link (由 robot_state_publisher 发布)
        ├── lidar_link   (fixed, z=0.1)
        ├── imu_link     (fixed, z=0.1)
        ├── left_front_wheel_joint
        ├── left_rear_wheel_joint
        ├── right_front_wheel_joint
        └── right_rear_wheel_joint
```

**重要**：当前 `map → odom` 由 FAST-LIO 发布。Nav2 需要这个 TF 链条，但可能要求 Nav2 接管或使用 `map` → `odom` 作为全局定位来源（通过 AMCL 或 FAST-LIO 的全局定位输出）。

### 4.3 机器人参数

| 参数 | 值 |
|------|-----|
| 底盘尺寸 | 0.4m × 0.3m × 0.15m |
| 底盘质量 | 10.0 kg |
| 轮距 (wheel_separation) | 0.36 m |
| 轮半径 (wheel_radius) | 0.08 m |
| 驱动方式 | 四轮差速滑移转向 (skid-steer) |
| 最大速度 (仿真) | 取决于 /cmd_vel 输入 |

---

## 五、当前桥接话题清单 (ros_gz_bridge)

| 桥接参数 | 方向 | ROS 话题 (remap 后) | 消息类型 | 用途 |
|---------|------|-------------------|---------|------|
| `/clock` | Ignition → ROS | `/clock` | `rosgraph_msgs/Clock` | 仿真时间同步 |
| `/lidar/points/points` | Ignition → ROS | `/velodyne_points` | `sensor_msgs/PointCloud2` | SLAM 输入 |
| `/imu/data` | Ignition → ROS | `/imu/data` | `sensor_msgs/Imu` | SLAM 输入 |
| `/model/outdoor_bot/cmd_vel` | ROS → Ignition | `/cmd_vel` | `geometry_msgs/Twist` | 速度控制 |
| `world/.../joint_state` | Ignition → ROS | `/joint_states` | `sensor_msgs/JointState` | RViz 模型渲染 |

---

## 六、FAST-LIO 配置要点

配置文件：`src/FAST_LIO_ROS2/config/mid360_sim.yaml`

| 参数 | 值 | 说明 |
|------|-----|------|
| `lidar_type` | 2 | Velodyne/PointCloud2 输入 |
| `scan_line` | 32 | 匹配降采样后的 32 线雷达 |
| `point_filter_num` | 3 | 每 3 个点采 1 个（降采样）|
| `filter_size_surf` | 0.15 | 面特征滤波尺寸 |
| `filter_size_map` | 0.15 | 地图体素滤波尺寸 |
| `extrinsic_T` | [0, 0, 0.03] | LiDAR→IMU 平移外参 |
| `extrinsic_R` | I₃ | 旋转为 Identity（已标定） |
| `extrinsic_est_en` | false | 关闭在线估计 |
| `time_sync_en` | false | 仿真时钟完美对齐 |
| `con_est_en` | false | 仿真无运动畸变 |
| `max_iteration` | 3 | 迭代次数（兼顾精度与性能）|

---

## 七、Nav2 集成需求（下一目标）

### 7.1 目标

在现有 FAST-LIO SLAM 建图的基础上，接入 **Nav2 (Navigation2)** 导航堆栈，实现：

1. **全局路径规划** — 给定目标点，规划从当前位置到目标的可行路径
2. **局部路径规划** — 实时避障，应对动态障碍物
3. **自主导航** — 四驱小车在封闭房间内自主移动到目标点

### 7.2 Nav2 需要的输入输出

| Nav2 需求 | 当前状态 | 差距分析 |
|-----------|---------|---------|
| **`/odom`** (nav_msgs/Odometry) | DiffDrive 发布 `/odom` (skid-steer odometry) | ✅ 已有，但质量一般（滑移转向轮式里程计漂移大） |
| **`/tf`**: `map → odom → base_link` | FAST-LIO 发布 `map → odom`，robot_state_pub 发布 `odom → base_link` | ⚠️ 需要确认 FAST-LIO 的 `map→odom` TF 是否满足 Nav2 要求 |
| **`/scan`** (sensor_msgs/LaserScan) | 只有 /velodyne_points (PointCloud2) | ❌ 需要 **pointcloud_to_laserscan** 节点转换，或 Nav2 直接消费 PointCloud2 (costmap 插件) |
| **`/map`** (nav_msgs/OccupancyGrid) | FAST-LIO 建图，但默认不发布 OccupancyGrid | ❌ 需要 **FAST-LIO 输出的点云地图 → 2D 代价地图** 转换 |
| **`/cmd_vel`** (geometry_msgs/Twist) | 已桥接到 Ignition DiffDrive | ✅ 已有 |
| **`/joint_states`** | 已桥接 | ✅ 最新已修复 |
| **初始位姿估计** | 需要提供初始位置和方向 | ❌ 需要实现 |

### 7.3 关键技术决策 (需架构师设计)

#### A. 全局定位方案

**选项 1：FAST-LIO 输出直接作为全局定位**
- FAST-LIO 发布 `map → odom` TF
- 优点：与当前 SLAM 一致，无额外计算
- 缺点：FAST-LIO 的 odom 帧可能随时间漂移（无回环检测），不好复位

**选项 2：在 FAST-LIO 地图上运行 AMCL**
- 将 FAST-LIO 点云地图 → 2D OccupancyGrid
- 在 2D 地图上运行 AMCL 粒子滤波定位
- 优点：标准 Nav2 方案，可重定位
- 缺点：LiDAR-only 的 AMCL 在退化环境（对称走廊）可能发散

**选项 3：FAST-LIO 输出作为里程计源 + Nav2 自己的全局定位**
- FAST-LIO odometry 代替轮式里程计作为 `/odom` 输入
- Nav2 接管全局定位

#### B. 2D 代价地图来源

FAST-LIO 输出的是 **3D 点云地图**，Nav2 需要 2D 代价地图。几条路径：

1. **点云投影** — 将 FAST-LIO 点云地图投影为 2D OccupancyGrid（高度切片 + 投影）
2. **Nav2 直接消费 3D** — 使用 `nav2_costmap_2d` 的 `PointCloud2` 层插件 (直接订阅 LiDAR 原始点云)
3. **PointCloud2 → LaserScan** — 使用 `pointcloud_to_laserscan` 将 3D 点云转换为 2D 激光扫描，供 Nav2 标准 costmap 使用

#### C. 代价地图配置

由于是室内 10m×10m 封闭房间 + 3 个柱子：
- 全局代价地图：10m×10m，分辨率 0.05m
- 局部代价地图：3m×3m 窗口，分辨率 0.05m
- 膨胀层：机器人半径 0.2m（底盘 0.4m×0.3m 的对角线约 0.25m）

### 7.4 需要新增的节点/包

| 功能 | 推荐包 | 说明 |
|------|--------|------|
| **导航堆栈** | `nav2_bringup` + `nav2_*` 组件 | ROS 2 Humble 官方 Nav2 |
| **点云转激光** | `pointcloud_to_laserscan` | 将 /velodyne_points → /scan |
| **SLAM 地图转 costmap** | 自定义节点 或 `map_server` | FAST-LIO PCD → OccupancyGrid |
| **路径规划** | Nav2 内置 (NavFn / Smac) | 全局规划器 |
| **动态避障** | Nav2 内置 (DWB / Regulated Pure Pursuit) | 局部规划器 |
| **行为树 XML** | `nav2_bt_navigator` | 定义导航行为 |
| **RViz 插件** | `nav2_rviz_plugins` | 2D Pose Estimate / 2D Goal Pose |

### 7.5 ARM64/RK3588 特定约束（给架构师的警告）

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     🚨 ARM64 部署警告 🚨                                 │
│                                                                          │
│  1. Nav2 的编译极其依赖资源                                              │
│     · nav2_* 包之间有复杂依赖链，colcon 自动解决                         │
│     · 确保 MAKEFLAGS="-j8"，否则 OOM 锁死                                │
│     · 预计第一次编译 15-30 分钟                                           │
│                                                                          │
│  2. 不可用 Nav2 的 GPU 加速插件                                          │
│     · GPU 软件渲染极度缓慢，禁用所有 GPU 视觉插件                         │
│     · costmap 使用 CPU 版本                                               │
│     · 禁用 nav2_collision_monitor 的 GPU 模块                            │
│                                                                          │
│  3. QoS 一致性                                                          │
│     · FAST-LIO 使用 BEST_EFFORT 接收 LiDAR/IMU                          │
│     · Nav2 costmap 默认 RELIABLE，可能导致 mismatch                      │
│     · 需要统一 Nav2 的传感器订阅也使用 BEST_EFFORT                        │
│                                                                          │
│  4. 仿真时间 vs 真实时间                                                 │
│     · 所有节点必须 use_sim_time=true                                     │
│     · Nav2 的 BT 超时参数需相应调整                                      │
│                                                                          │
│  5. 点云数据量                                                          │
│     · 32 线 × 400 点每线 = 12,800 点/帧，对 RK3588 可接受                │
│     · pointcloud_to_laserscan 会进一步降采样                             │
│     · 如果性能不够，可将 horizontal samples 从 400 降至 200               │
│                                                                          │
│  6. 内存预算                                                             │
│     · Ignition Fortress (server only): ~600MB                           │
│     · FAST-LIO SLAM: ~200-400MB                                          │
│     · Nav2 全套: ~200-300MB                                              │
│     · 总计约 1.2-1.5 GB，16GB 绰绰有余                                   │
│                                                                          │
│  7. 已安装的 Nav2 相关包                                                 │
│     · 可通过 ros2 pkg list | grep nav2 检查已安装的 Nav2 组件            │
│     · 缺失的用 sudo apt install ros-humble-xxx 安装                      │
└─────────────────────────────────────────────────────────────────────────┘
```

### 7.6 建议的 Nav2 启动流程

```
1. gz sim -s -r grass_terrain.world           # 启动仿真世界
2. robot_state_publisher                      # 发布 TF
3. ros_gz_sim create                          # 生成机器人
4. ros_gz_bridge parameter_bridge             # 桥接话题
5. FAST-LIO (mid360_sim.yaml)                 # SLAM 建图
6. pointcloud_to_laserscan                    # PointCloud2 → LaserScan
7. map_saver/map_server                       # 保存/加载地图
8. nav2_bringup                               # 启动导航堆栈
9. RViz2 + Nav2 面板                          # 可视化 + 目标下达
```

---

## 八、当前项目文件结构

```
~/ros2_ws/
├── src/
│   ├── outdoor_sim/                          # 仿真包
│   │   ├── worlds/grass_terrain.world        # SDF 世界 (10m×10m 房间)
│   │   ├── urdf/robot_sensors.xacro          # 机器人 URDF 描述
│   │   ├── launch/sim_launch.py              # 主启动文件
│   │   └── package.xml                       # 依赖声明
│   │
│   └── FAST_LIO_ROS2/                        # SLAM 建图包
│       ├── config/mid360_sim.yaml            # 仿真专用配置
│       ├── launch/                           # FAST-LIO 启动文件
│       ├── src/                              # C++ 源码 (C++17, NEON)
│       ├── include/                          # 头文件
│       └── package.xml
│
├── CLAUDE.md                                 # 项目规则文档
├── project_overview_for_gemini.md            ← 本文件
└── (build/, install/, log/ — 均为 gitignored)
```

---

## 九、启动命令速查

```bash
# 1. 设置环境变量（每次新终端都执行）
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export MAKEFLAGS="-j8"
source ~/ros2_ws/install/setup.bash

# 2. 启动仿真（Headless 模式）
export LIBGL_ALWAYS_SOFTWARE=1
xvfb-run -a ros2 launch outdoor_sim sim_launch.py

# 3. 启动 FAST-LIO
ros2 launch fast_lio mapping.launch.py config:=mid360_sim.yaml

# 4. 保存地图
ros2 run fast_lio save_map --save_path /home/orangepi/ros2_ws/my_3d_map.pcd

# 5. 检查话题
ros2 topic list
ros2 topic hz /velodyne_points
ros2 topic hz /imu/data

# 6. 控制小车（单独终端）
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"

# 7. 查看 TF 树
ros2 run tf2_tools view_frames
```

---

## 十、当前已知问题（供架构师参考）

| # | 问题 | 状态 | 说明 |
|---|------|------|------|
| 1 | LiDAR 从 64 线降至 32 线 | ✅ 已优化 | 降低 CPU 防止掉帧 |
| 2 | 车轮 3D 模型在 RViz 不可见 | ✅ 已修复 | 追加 JointStatePublisher + 桥接 |
| 3 | 室内 10m×10m 封闭环境 | ✅ 已实现 | 四面墙 + 天花板 + 3 柱子 |
| 4 | FAST-LIO 外参标定 | ✅ 已修正 | extrinsic_T=[0,0,0.03] |
| 5 | 没有 2D 地图供 Nav2 | ❌ 待解决 | 需将 PCD 地图转 OccupancyGrid |
| 6 | 没有 /scan 话题 | ❌ 待解决 | 需 pointcloud_to_laserscan |
| 7 | 轮式里程计漂移大 | ⚠️ 已知 | 滑移转向先天不足，建议用 FAST-LIO odometry 替代 |
| 8 | 没有回环检测 | ⚠️ 已知 | FAST-LIO 无回环，长时间运行地图会漂移 |
| 9 | Nav2 尚未安装 | ❌ 待解决 | `sudo apt install ros-humble-nav2-*` |

---

> **给 Gemini 架构师的核心问题：**
> 基于现有 FAST-LIO SLAM + Ignition Fortress 仿真平台，请设计 Nav2 集成方案，包括：
> 1. 全局定位方案选择（FAST-LIO direct vs AMCL vs hybrid）
> 2. 2D 代价地图生成策略（3D→2D 投影 / 直接消费 PointCloud2 / LaserScan 转换）
> 3. 需要的新增节点、配置文件和启动脚本清单
> 4. ARM64 特定优化建议（编译、QoS、性能调优）
> 5. 具体的参数配置建议（costmap 尺寸、规划器选型、BT 行为树）
> 6. 实施步骤（按顺序列出从安装到验证的每一步）
