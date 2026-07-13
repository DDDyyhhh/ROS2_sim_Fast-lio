# Outdoor Bot — 4WD LiDAR SLAM & Navigation Simulation

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-34a058)](https://docs.ros.org/en/humble/)
[![Ignition](https://img.shields.io/badge/Ignition-Fortress-ff69b4)](https://ignitionrobotics.org/)
[![Platform](https://img.shields.io/badge/Platform-ARM64%20(RK3588)-blue)](https://www.rock-chips.com/)
[![SLAM](https://img.shields.io/badge/SLAM-FAST--LIO%202-orange)](https://github.com/hku-mars/FAST_LIO)
[![Nav2](https://img.shields.io/badge/Nav2-Integrated-success)](https://navigation.ros.org/)

> **基于 Orange Pi 5 Plus (RK3588) 的 ROS 2 Humble 仿真平台**
> 四驱差速小车 + 32线 LiDAR + 200Hz IMU + FAST-LIO SLAM + **Nav2 自主导航**
> 支持两种工作模式：纯导航（预建地图 + AMCL）与 SLAM+导航（未知环境探索）

---

## 目录

- [项目概述](#项目概述)
- [硬件要求](#硬件要求)
- [软件技术栈](#软件技术栈)
- [快速开始](#快速开始)
- [系统架构](#系统架构)
  - [数据流架构](#数据流架构)
  - [TF 树](#tf-树)
  - [桥接话题清单](#桥接话题清单)
- [两种 Nav2 工作模式](#两种-nav2-工作模式)
- [用法指南](#用法指南)
- [PCD→PGM 地图转换工具](#pcdpgm-地图转换工具)
- [项目结构](#项目结构)
- [ARM64 特定注意事项](#arm64-rk3588-特定注意事项)
- [已知限制](#已知限制)
- [故障排除](#故障排除)
- [许可证](#许可证)

---

## 项目概述

本项目在 **Orange Pi 5 Plus**（RK3588, ARM64 单板计算机）上搭建了一套完整的 ROS 2 仿真与自主导航系统：

- **仿真平台**：Ignition Fortress (Gazebo Sim 6.x)，SDF 1.8，10m×10m 封闭室内环境
- **机器人模型**：四轮差速滑移转向小车 (4WD skid-steer)，0.4m×0.3m 底盘
- **传感器套件**：
  - 32线 360° LiDAR（等效 DJI Mid-360），10 Hz，0.1–40m 测距
  - 6轴 IMU（含高斯噪声模型），200 Hz
- **SLAM 建图**：FAST-LIO 2（实时 LiDAR-IMU 融合里程计与建图）
- **自主导航**：Nav2 导航堆栈，**两种工作模式**均已集成：
  - **模式 A**：纯导航 — 预建 PGM 地图 + AMCL 粒子滤波定位 + Nav2 全局/局部规划
  - **模式 B**：SLAM+导航 — 未知环境同时建图与导航，FAST-LIO 实时里程计驱动 Nav2

所有配置针对 **ARM64 架构** 深度优化，绕开了 RK3588 Mali-G610 GPU 驱动问题和多核编译死锁风险。

---

## 硬件要求

| 组件 | 最低要求 | 推荐 |
|------|---------|------|
| **主板** | Orange Pi 5 / 5 Plus | Orange Pi 5 Plus |
| **RAM** | 8 GB | 16 GB |
| **存储** | 32 GB eMMC/SD | 64 GB+ |
| **OS** | Ubuntu 22.04 LTS (aarch64) | Ubuntu 22.04 LTS Server |

> **注意**：本项目针对 **ARM64 (aarch64)** 架构深度定制。在 x86_64 上运行时需调整编译选项和 GPU 渲染配置。

---

## 软件技术栈

| 层次 | 技术选型 | 版本 |
|------|---------|------|
| **操作系统** | Ubuntu | 22.04 LTS (Jammy) |
| **中间件** | ROS 2 | Humble Hawksbill |
| **DDS** | Eclipse CycloneDDS | （默认 FastRTPS 不兼容 RK3588） |
| **仿真引擎** | Ignition Fortress (Gazebo Sim) | 6.x, SDF 1.8 |
| **仿真桥接** | ros_gz_bridge / ros_gz_sim | Humble 官方 |
| **SLAM 建图** | FAST-LIO 2 (`fast_lio`) | 自编译 aarch64 |
| **导航堆栈** | Nav2 (nav2_bringup) | Humble 官方 |
| **URDF 解析** | robot_state_publisher + xacro | Humble 官方 |
| **可视化** | RViz2 + nav2_rviz_plugins | Humble 官方 |
| **点云转激光** | pointcloud_to_laserscan | Humble 官方 |
| **C++ 标准** | C++17, aarch64 NEON 优化 | `-march=armv8-a+crypto -mcpu=cortex-a76` |

---

## 快速开始

### 1. 安装依赖

```bash
# ROS 2 Humble
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

# CycloneDDS（推荐，避免 RK3588 DDS 问题）
sudo apt install -y ros-humble-rmw-cyclonedds-cpp

# Ignition Fortress
sudo apt install -y ignition-fortress
sudo apt install -y ros-humble-ros-gz

# FAST-LIO 依赖
sudo apt install -y libpcl-dev libeigen3-dev libyaml-cpp-dev

# Nav2 及辅助工具
sudo apt install -y ros-humble-nav2-* ros-humble-navigation2
sudo apt install -y ros-humble-pointcloud-to-laserscan
sudo apt install -y ros-humble-teleop-twist-keyboard
```

### 2. 克隆并编译

```bash
git clone git@github.com:DDDyyhhh/ROS2_sim_Fast-lio.git /home/yh/mower_ws
cd /home/yh/mower_ws

# 环境变量（每次新终端都执行）
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export MAKEFLAGS="-j8"

# 编译
colcon build --symlink-install
source install/setup.bash
```

---

## 系统架构

### 数据流架构

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         Ignition Fortress 仿真世界                            │
│                                                                             │
│   World: grass_terrain.world    (10m×10m 封闭房间 + 3 柱子障碍物)            │
│   Robot: outdoor_bot (4WD skid-steer)                                       │
│   ├─ DiffDrivePlugin    → /odom          (20Hz 里程计)                      │
│   ├─ LiDAR (32线, 10Hz) → /lidar/points  (点云)                             │
│   ├─ IMU (200Hz)        → /imu/data      (6轴惯导)                          │
│   └─ /model/outdoor_bot/cmd_vel ← 速度命令                                  │
└───────────────────────────────────┬─────────────────────────────────────────┘
                                    │ ros_gz_bridge
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              ROS 2 层                                        │
│                                                                              │
│  /velodyne_points, /imu/data, /odom, /clock, /joint_states, /cmd_vel        │
│                                                                              │
│  ┌───────────────────────────────────────────────────────────────────┐      │
│  │  模式 A: 纯导航 (nav2_sim_launch.py)                              │      │
│  │  ┌───────────────────────────────────────────────────────────┐   │      │
│  │  │  pointcloud_to_laserscan → /scan                          │   │      │
│  │  │  Nav2 (map_server + AMCL + DWB planner)                   │   │      │
│  │  │  RViz2 (Navigation 2 Panel)                               │   │      │
│  │  └───────────────────────────────────────────────────────────┘   │      │
│  └───────────────────────────────────────────────────────────────────┘      │
│                                                                              │
│  ┌───────────────────────────────────────────────────────────────────┐      │
│  │  模式 B: SLAM+导航 (slam_nav2_launch.py)                          │      │
│  │  ┌───────────────────────────────────────────────────────────┐   │      │
│  │  │  FAST-LIO SLAM (实时里程计)                                 │   │      │
│  │  │  静态 TF: map → odom → camera_init                         │   │      │
│  │  │  pointcloud_to_laserscan → /scan                           │   │      │
│  │  │  Nav2 (无 AMCL, 无 map_server)                             │   │      │
│  │  │  RViz2 (Navigation 2 Panel)                                │   │      │
│  │  └───────────────────────────────────────────────────────────┘   │      │
│  └───────────────────────────────────────────────────────────────────┘      │
└─────────────────────────────────────────────────────────────────────────────┘
```

### TF 树

**模式 A — 纯导航（预建地图 + AMCL）：**
```
map ──(AMCL)──→ odom ──(DiffDrive/robot_state_publisher)──→ body
                                                              ├── lidar_link
                                                              ├── imu_link
                                                              └── wheel_* (4×)
```

**模式 B — SLAM+导航（FAST-LIO 实时里程计）：**
```
map ──(static)──→ odom ──(static)──→ camera_init ──(FAST-LIO)──→ body
```

### 桥接话题清单

| Ignition 话题 | 方向 | ROS 话题 | 类型 | 频率 | 用途 |
|-------------|------|---------|------|------|------|
| `/clock` | → | `/clock` | `rosgraph_msgs/Clock` | — | 仿真时间同步 |
| `/lidar/points/points` | → | `/velodyne_points` | `sensor_msgs/PointCloud2` | 10 Hz | LiDAR 点云 |
| `/imu/data` | → | `/imu/data` | `sensor_msgs/Imu` | 200 Hz | IMU 数据 |
| `/odom` | → | `/odom` | `nav_msgs/Odometry` | 20 Hz | DiffDrive 里程计 |
| `/world/.../joint_state` | → | `/joint_states` | `sensor_msgs/JointState` | 30 Hz | 车轮转动 |
| `/model/outdoor_bot/cmd_vel` | ← | `/cmd_vel` | `geometry_msgs/Twist` | — | 速度控制 |

---

## 两种 Nav2 工作模式

| 模式 | 启动文件 | 参数文件 | 适用场景 | 定位方式 |
|------|---------|---------|---------|---------|
| **A: 纯导航** | `nav2_sim_launch.py` | `nav2_params.yaml` | 已知环境，需要精确导航 | AMCL 粒子滤波 + 预建地图 |
| **B: SLAM+导航** | `slam_nav2_launch.py` | `nav2_slam_params.yaml` | 未知环境探索 | FAST-LIO 实时里程计 |

### 模式 A：纯导航（预建地图 + AMCL）

经典的两步式流程：先建图 → 保存 PCD → 转为 PGM → 启动 Nav2。

**AMCL 参数亮点：**
- `max_particles`: 2000（降粒子数以节省 RK3588 CPU）
- `base_frame_id`: `body`（⚠️ 非 `base_link`，已全局对齐）
- `alpha1/alpha4`: 1.5（滑移转向自转时里程计模型高噪声）
- `laser_model_type`: `likelihood_field`（比 beam 模型快）

**DWB 控制器亮点：**
- `max_vel_x`: 0.5 m/s，`max_vel_theta`: 1.0 rad/s
- `yaw_goal_tolerance`: 0.30 rad（终点角度容差~17°）
- `RotateToGoal.slowing_factor`: 2.5（防终点卡死）
- `movement_time_allowance`: 15.0 s（调向超时窗口）

### 模式 B：SLAM+导航（未知环境探索）

一键式启动，FAST-LIO 和 Nav2 同时运行：

- **FAST-LIO** 提供实时里程计（`map → camera_init → body`）
- Nav2 使用 obstacle_layer（无 static_map）做实时避障
- 全局代价地图 **100×100m** 大画布
- **无 AMCL**、**无 map_server**

**与模式 A 关键参数差异：**
- 全局地图：static_map + inflation → obstacle_layer + inflation
- `min_vel_x`: 0.0 → **-0.15**（允许倒车微调）
- `xy_goal_tolerance`: 0.25 → **0.50 m**（漂移容差翻倍）
- `yaw_goal_tolerance`: 0.30 → **0.45 rad**

---

## 用法指南

### 一键启动命令速查

```bash
# ── 0. 环境变量（每次新终端都执行） ───────────────
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export MAKEFLAGS="-j8"
source /home/yh/mower_ws/install/setup.bash

# ── 1. 启动仿真（Headless） ─────────────────────
export LIBGL_ALWAYS_SOFTWARE=1
xvfb-run -a ros2 launch outdoor_sim sim_launch.py

# ── 2. 模式 A：纯导航（需先建图） ─────────────────
#    建图完成后，关掉 FAST-LIO，然后：
ros2 launch outdoor_sim nav2_sim_launch.py

# ── 3. 模式 B：SLAM+导航（一键启动） ─────────────
#    仿真运行后，另一个终端：
ros2 launch outdoor_sim slam_nav2_launch.py
```

### 建图流程（模式 A 前置步骤）

```bash
# 终端 1：启动仿真
export LIBGL_ALWAYS_SOFTWARE=1
xvfb-run -a ros2 launch outdoor_sim sim_launch.py

# 终端 2：启动 FAST-LIO SLAM
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /home/yh/mower_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py config:=mid360_sim.yaml

# 终端 3：手动控制小车遍历环境
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.2}}"

# 保存 PCD 点云地图
ros2 run fast_lio save_map --save_path /home/yh/mower_ws/my_3d_map.pcd

# 转换为 Nav2 PGM 地图
python3 /home/yh/mower_ws/pcd_to_pgm.py

# 关掉 FAST-LIO，启动 Nav2 纯导航
ros2 launch outdoor_sim nav2_sim_launch.py
```

### 启动仿真

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /home/yh/mower_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

# 有桌面环境
ros2 launch outdoor_sim sim_launch.py

# 无桌面环境 (SSH / 纯终端)
xvfb-run -a ros2 launch outdoor_sim sim_launch.py
```

### 控制小车

```bash
# 手动发布速度命令
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" -1

# 或使用键盘
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 检查系统状态

```bash
# 话题列表
ros2 topic list

# 检查频率
ros2 topic hz /velodyne_points /imu/data /odom /scan

# TF 树
ros2 run tf2_tools view_frames
```

---

## PCD→PGM 地图转换工具

**位置**：项目根目录 `pcd_to_pgm.py`

将 FAST-LIO 保存的 3D `.pcd` 点云地图转换为 Nav2 可用的 2D 栅格地图：

```
my_3d_map.pcd (FAST-LIO)
    ↓ python3 pcd_to_pgm.py
my_3d_map.pgm + my_3d_map.yaml (Nav2 map_server)
```

**转换参数**：Z 轴切片 0.1–1.5m，分辨率 0.05 m/px，障碍物膨胀 0.25m。

---

## 项目结构

```
/home/yh/mower_ws/
├── src/
│   ├── outdoor_sim/                          # 仿真包
│   │   ├── worlds/grass_terrain.world        # SDF 1.8 世界文件
│   │   ├── urdf/robot_sensors.xacro          # 机器人 URDF
│   │   ├── launch/
│   │   │   ├── sim_launch.py                 # 主仿真启动
│   │   │   ├── nav2_sim_launch.py            # 模式 A：纯导航
│   │   │   └── slam_nav2_launch.py           # 模式 B：SLAM+导航
│   │   ├── config/
│   │   │   ├── nav2_params.yaml              # 模式 A 参数
│   │   │   ├── nav2_slam_params.yaml         # 模式 B 参数
│   │   │   └── nav2_3d_view.rviz             # Nav2 RViz 配置
│   │   ├── scripts/
│   │   │   ├── odom_to_tf.py                 # /odom→/tf 中继
│   │   │   ├── clock_filter.py               # 时钟回跳滤波
│   │   │   └── tf_waiter.py                  # TF 等待辅助
│   │   └── CMakeLists.txt + package.xml
│   │
│   ├── fast_lio/                             # SLAM 建图包
│   │   ├── config/mid360_sim.yaml            # 仿真配置
│   │   ├── launch/ + src/ + include/
│   │   └── package.xml
│   │
│   ├── pcd_to_pgm.py                         # PCD→PGM 转换
│   └── CLAUDE.md + project_overview_for_gemini.md
│
├── build/ install/ log/                      # gitignored
└── README.md                                 ← 本文件
```

---

## ARM64 (RK3588) 特定注意事项

| 问题 | 影响 | 对策 |
|------|------|------|
| Mali-G610 GPU 驱动不稳定 | Ogre2 segfault (exit -11) | `LIBGL_ALWAYS_SOFTWARE=1` 软件渲染；世界文件用 `ogre` 引擎 |
| 8 核 ARM 多线程竞争 | colcon 编译锁死 | `export MAKEFLAGS="-j8"` |
| 有限内存带宽 | RELIABLE QoS 导致 DDS 溢出 | LiDAR/IMU 强制 `BEST_EFFORT` |
| Heightmap shader 不兼容 | Ogre shader 崩溃 | 禁止 heightmap，改用 `<box>` |
| 机器人 frame id | 配置对齐要求 | 全部使用 `body`（非 `base_link`） |

**内存预算**：Ignition ~600MB + FAST-LIO ~400MB + Nav2 ~300MB ≈ 1.3 GB（16GB 绰绰有余）

---

## 已知限制

| 类别 | 限制 | 说明 |
|------|------|------|
| **GPU** | 禁用硬件加速 | Mali-G610 驱动不稳定，使用软件渲染 |
| **编译** | 最大 8 线程 | `MAKEFLAGS="-j8"`，否则 OOM 锁死 |
| **DDS** | 强制 CycloneDDS | FastRTPS 在 RK3588 上偶发崩溃 |
| **LiDAR** | 32 线上限 | 64 线导致 RK3588 CPU 掉帧 |
| **QoS** | 传感器必须是 BEST_EFFORT | RELIABLE 导致 DDS 缓冲溢出 |
| **SLAM** | 无回环检测 | FAST-LIO 2 不含回环模块，长时间运行会漂移 |
| **轮式里程计** | 滑移转向漂移大 | 用 FAST-LIO odometry 替代可改善 |
| **模式 A 地图质量** | PCD→PGM 尚需验证 | Z 轴切片参数可能需要微调 |
| **模式 B 代价地图** | 无静态层 | 仅 obstacle_layer，空旷区域路径可能不可靠 |

---

## 故障排除

### 仿真卡住/无输出

```bash
ps aux | grep gz        # Gazebo 是否运行
ros2 topic list          # 话题是否存在
ros2 topic echo /clock --once  # 时钟是否前进
```

### LiDAR 无点云

```bash
ros2 topic hz /velodyne_points  # 频率是否为 0
# 确认 xacro 中传感器类型为 gpu_lidar
```

### 编译失败

```bash
rm -rf build/ install/ log/
colcon build --symlink-install --cmake-clean-first
```

### FAST-LIO 发散

```bash
# 检查 mid360_sim.yaml 中外参 extrinsic_T = [0, 0, 0.03]
# 确认 scan_line: 32 与 xacro 一致
# 降低车速 (x: 0.3 以下)
```

### Nav2 不规划

```bash
ros2 run tf2_tools view_frames   # TF 树完整性
grep -r "body" src/outdoor_sim/config/  # frame id 对齐
ros2 topic echo /scan --once     # 激光数据是否正常
```

---

## 许可证

本项目基于 Apache 2.0 许可证开源。FAST-LIO 部分遵循其上游 GPL-2.0 许可证。

---

## 参考文献

- [FAST-LIO 2](https://github.com/hku-mars/FAST_LIO)
- [Ignition Fortress 文档](https://ignitionrobotics.org/docs/fortress)
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)
- [Nav2 文档](https://navigation.ros.org/)
- [Orange Pi 5 Plus 规格](http://www.orangepi.org/html/hardWare/computerAndMicrocontrollers/details/Orange-Pi-5-Plus.html)
- [Navigation 2 Tuning Guide](https://navigation.ros.org/tuning/index.html)
