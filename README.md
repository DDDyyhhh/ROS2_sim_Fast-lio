# Outdoor Bot — 4WD LiDAR SLAM & Navigation Simulation

[![ROS 2](https://img.shields.io/badge/ROS2-Humble-34a058)](https://docs.ros.org/en/humble/)
[![Ignition](https://img.shields.io/badge/Ignition-Fortress-ff69b4)](https://ignitionrobotics.org/)
[![Platform](https://img.shields.io/badge/Platform-ARM64%20(RK3588)-blue)](https://www.rock-chips.com/)
[![LiDAR](https://img.shields.io/badge/LiDAR-32--line%20Mid--360%20equivalent-important)]()

> **基于 Orange Pi 5 Plus (RK3588) 的 ROS 2 Humble 仿真平台**
> 四驱差速小车 + 32线 LiDAR + 200Hz IMU + FAST-LIO SLAM + Ignition Fortress
> 目标：在仿真环境中验证自主导航算法，为真车部署做准备

---

## 目录

- [项目概述](#项目概述)
- [硬件要求](#硬件要求)
- [快速开始](#快速开始)
- [系统架构](#系统架构)
- [用法指南](#用法指南)
  - [启动仿真](#启动仿真)
  - [启动 SLAM 建图](#启动-slam-建图)
  - [控制小车](#控制小车)
  - [保存地图](#保存地图)
  - [可视化 (RViz2)](#可视化-rviz2)
- [导航堆栈 (Nav2)](#导航堆栈-nav2)
- [项目结构](#项目结构)
- [已知限制](#已知限制)
- [故障排除](#故障排除)

---

## 项目概述

本项目在 **Orange Pi 5 Plus**（RK3588, ARM64）上搭建了一套完整的 ROS 2 仿真系统：

- **仿真平台**：Ignition Fortress (Gazebo Sim 6.x)，SDF 1.8
- **机器人模型**：四轮差速滑移转向小车 (4WD skid-steer)
- **传感器套件**：
  - 32线 360° LiDAR（等效 DJI Mid-360），10 Hz，40m 测距
  - 6轴 IMU（含高斯噪声模型），200 Hz
- **SLAM 建图**：FAST-LIO 2 (实时 LiDAR-IMU 融合里程计与建图)
- **下一步**：接入 Nav2 导航堆栈，实现自主避障与路径规划

所有配置针对 **ARM64 架构**优化，避开了 Mali-G610 GPU 驱动问题和多核编译死锁风险。

---

## 硬件要求

| 组件 | 最低要求 | 推荐 |
|------|---------|------|
| **主板** | Orange Pi 5 / 5 Plus | Orange Pi 5 Plus |
| **RAM** | 8 GB | 16 GB |
| **存储** | 32 GB eMMC/SD | 64 GB+ |
| **OS** | Ubuntu 22.04 LTS (aarch64) | Ubuntu 22.04 LTS Server |

> **注意**：本项目针对 **ARM64 (aarch64)** 架构深度定制。在 x86_64 上运行时需调整编译选项和 GPU 配置。

---

## 快速开始

### 1. 安装 ROS 2 Humble

```bash
# 安装 ROS 2 Humble (Ubuntu 22.04 ARM64)
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

# 安装 CycloneDDS（推荐，避免 RK3588 上的 DDS 问题）
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
```

### 2. 安装 Ignition Fortress

```bash
sudo apt install -y ignition-fortress
sudo apt install -y ros-humble-ros-gz
```

### 3. 安装 FAST-LIO 依赖

```bash
sudo apt install -y libpcl-dev libeigen3-dev libyaml-cpp-dev
```

### 4. 克隆并编译

```bash
git clone https://github.com/your-org/outdoor-bot.git ~/ros2_ws
cd ~/ros2_ws

# 环境变量（每次新终端都执行）
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export MAKEFLAGS="-j8"

# 编译
colcon build --symlink-install
source install/setup.bash
```

---

## 系统架构

### 数据流

```
┌─────────────── Ignition Fortress ───────────────┐
│                                                   │
│   10m×10m 室内封闭房间              ROS 2          │
│   (四面墙 + 天花板 + 3 柱子)          │             │
│                                          │             │
│   Robot: outdoor_bot (4WD)              │             │
│   ├─ DiffDrivePlugin → /odom ──── ros_gz_bridge ─→ /odom
│   ├─ LiDAR (32线)    → /lidar/points ─ ros_gz_bridge ─→ /velodyne_points ─→ FAST-LIO
│   ├─ IMU (200Hz)    → /imu/data ──── ros_gz_bridge ─→ /imu/data ────────→ FAST-LIO
│   ├─ JointStatePub  → /joint_state  ─ ros_gz_bridge ─→ /joint_states ──→ RViz2
│   └─ cmd_vel ←──── ros_gz_bridge ←─── /cmd_vel ←── Nav2 (未来)
│                                                   │
└───────────────────────────────────────────────────┘
```

### TF 树

```
map ← FAST-LIO
  └── odom ← FAST-LIO / DiffDrive
        └── body (base_link)
              ├── lidar_link
              ├── imu_link
              ├── left_front_wheel
              ├── left_rear_wheel
              ├── right_front_wheel
              └── right_rear_wheel
```

### 桥接话题清单

| ROS 话题 | 方向 | 类型 | 频率 | 用途 |
|----------|------|------|------|------|
| `/velodyne_points` | Ignition → ROS | `sensor_msgs/PointCloud2` | 10 Hz | LiDAR 点云 |
| `/imu/data` | Ignition → ROS | `sensor_msgs/Imu` | 200 Hz | IMU 数据 |
| `/clock` | Ignition → ROS | `rosgraph_msgs/Clock` | — | 仿真时钟 |
| `/cmd_vel` | ROS → Ignition | `geometry_msgs/Twist` | — | 速度控制 |
| `/joint_states` | Ignition → ROS | `sensor_msgs/JointState` | 30 Hz | 车轮状态 |

---

## 用法指南

### 启动仿真

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/ros2_ws/install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

# 有桌面环境
ros2 launch outdoor_sim sim_launch.py

# 无桌面环境 (SSH / 纯终端)
xvfb-run -a ros2 launch outdoor_sim sim_launch.py
```

预期输出（首个终端）：
```
[INFO] [gz_sim-1]: process started with pid [12345]
...
[INFO] [ros_gz_bridge-4]: Connected to Ignition Gazebo...
```

### 启动 SLAM 建图

第二个终端：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/ros2_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py config:=mid360_sim.yaml
```

### 控制小车

第三个终端：

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/ros2_ws/install/setup.bash

# 直线前进
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" -1

# 原地旋转
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" -1

# 走一个方形路径（依次执行）
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}" -1
sleep 2
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" -1
sleep 1.6
# 重复... (也可用 keyboard teleop)
```

或使用键盘控制：

```bash
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 保存地图

```bash
# 保存 PCD 点云地图
ros2 run fast_lio save_map --save_path /home/orangepi/ros2_ws/my_3d_map.pcd

# 或使用辅助脚本
python3 save_map.py
```

### 可视化 (RViz2)

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/ros2_ws/install/setup.bash
rviz2
```

推荐加载的显示项：
- `RobotModel`（订阅 `/robot_description`）
- `PointCloud2`（订阅 `/velodyne_points`）
- `Map`（订阅 `/fastlio_mapping/global_map`，若 FAST-LIO 已启动）
- `TF`（查看坐标变换树）

---

## 导航堆栈 (Nav2)

> ⚠️ 当前仿真平台已就绪，**Nav2 集成正在规划中**。
> 详细架构设计文档见 [`project_overview_for_gemini.md`](project_overview_for_gemini.md)。

### 预期的 Nav2 数据流

```
FAST-LIO SLAM ──→ map (3D PCD)
                      ↓
              pointcloud_to_laserscan ──→ /scan (LaserScan)
                      ↓
              map_server / nav2_map_server ──→ /map (OccupancyGrid)
                      ↓
              Nav2 stack (global + local planner)
                      ↓
              /cmd_vel ──→ Ignition DiffDrive
```

### 下一步待办

- [ ] 安装 Nav2: `sudo apt install ros-humble-nav2-*`
- [ ] 安装 `pointcloud_to_laserscan`
- [ ] 配置 costmap 参数 (室内 10m×10m，分辨率 0.05m)
- [ ] 配置全局/局部规划器
- [ ] 创建 Nav2 行为树 XML
- [ ] 编写导航启动文件
- [ ] 集成 FAST-LIO odom 作为里程计源
- [ ] 实地测试避障和路径规划

---

## 项目结构

```
~/ros2_ws/
├── src/
│   ├── outdoor_sim/                    # 仿真包
│   │   ├── worlds/                     # Ignition SDF 世界文件
│   │   │   └── grass_terrain.world     # 10m×10m 封闭房间 + 3 柱子
│   │   ├── urdf/                       # 机器人 URDF/Xacro 描述
│   │   │   └── robot_sensors.xacro     # 4WD 底盘 + LiDAR + IMU
│   │   └── launch/                     # ROS 2 启动文件
│   │       └── sim_launch.py           # 主启动 (仿真 + 桥接 + 机器人)
│   │
│   └── FAST_LIO_ROS2/                  # FAST-LIO SLAM 建图包
│       ├── config/
│       │   └── mid360_sim.yaml         # 仿真专用配置 (32线, 外参)
│       ├── launch/
│       │   └── mapping.launch.py       # SLAM 启动文件
│       ├── src/                        # C++ 源码 (C++17, NEON 优化)
│       └── include/                    # 头文件及 IKFoM 工具包
│
├── CLAUDE.md                           # 项目开发规则 (for Claude Code)
├── project_overview_for_gemini.md      # Nav2 架构设计文档
└── save_map.py                         # PCD 地图保存脚本
```

---

## 已知限制

| 类别 | 限制 | 说明 |
|------|------|------|
| **GPU** | 禁用硬件加速 | Mali-G610 驱动不稳定，`LIBGL_ALWAYS_SOFTWARE=1` 软件渲染 |
| **编译** | 最大 8 线程 | `MAKEFLAGS="-j8"`，否则 OOM 锁死 |
| **DDS** | 强制 CycloneDDS | FastRTPS 在 RK3588 上偶发崩溃 |
| **Heightmap** | 禁止使用 | Ogre shader segfault (exit -11)，改用 `box` 几何体 |
| **LiDAR** | 32 线上限 | 64 线会导致 RK3588 CPU 掉帧，SLAM 发散 |
| **QoS** | 传感器必须是 BEST_EFFORT | RELIABLE 导致 DDS 缓冲溢出 |
| **SLAM** | 无回环检测 | FAST-LIO 2 不含回环模块，长时间运行地图会漂移 |
| **轮式里程计** | 滑移转向漂移大 | 用 FAST-LIO odometry 替代可改善 |

---

## 故障排除

### 仿真卡住/无输出

```bash
# 检查 Gazebo 服务器是否在运行
ps aux | grep gz
# 检查话题
ros2 topic list
# 检查时钟是否前进
ros2 topic echo /clock --once
```

### LiDAR 无点云

```bash
ros2 topic hz /velodyne_points
# 如果为 0，检查 Ignition 传感器插件是否加载
# 在 xacro 中确认传感器类型为 gpu_lidar（不是 CPU lidar）
```

### 编译失败

```bash
# 清理并重试
rm -rf build/ install/ log/
colcon build --symlink-install --cmake-clean-first
```

### RViz2 看不到小车

```bash
# 检查 joint_states 话题
ros2 topic echo /joint_states --once
# 确认 robot_state_publisher 在运行
ros2 node list | grep robot_state
```

### FAST-LIO 发散

```bash
# 检查外参配置
# mid360_sim.yaml 中 extrinsic_T 应为 [0, 0, 0.03]
# 确认 scan_line: 32 与 xacro 中的 samples 一致
# 降低速度 (cmd_vel x: 0.3 以下) 避免运动畸变
```

---

## 许可证

本项目基于 Apache 2.0 许可证开源。FAST-LIO 部分遵循其上游许可证 (GPL-2.0)。

---

## 参考文献

- [FAST-LIO 2 (GitHub)](https://github.com/hku-mars/FAST_LIO)
- [Ignition Fortress 文档](https://ignitionrobotics.org/docs/fortress)
- [ROS 2 Humble 文档](https://docs.ros.org/en/humble/)
- [Nav2 文档](https://navigation.ros.org/)
- [Orange Pi 5 Plus 规格](http://www.orangepi.org/html/hardWare/computerAndMicrocontrollers/details/Orange-Pi-5-Plus.html)
