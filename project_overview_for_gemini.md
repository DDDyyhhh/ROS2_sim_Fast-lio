# Orange Pi 5 Plus — Autonomous 4WD LiDAR SLAM & Navigation Project

> **给 Gemini 架构师的项目总览文档**
> 目标：在仿真环境中为四驱差速小车接入 Nav2 导航堆栈，实现自主避障与路径规划。

---

## 一、项目背景

本项目在 **Orange Pi 5 Plus**（RK3588, ARM64）单板计算机上，搭建了一套完整的 **ROS 2 Humble** + **Ignition Fortress** 仿真系统，模拟一台装备 **DJI Mid-360 等效 LiDAR** + **200Hz IMU** 的四驱差速小车，在封闭室内环境中进行 **FAST-LIO SLAM** 建图与 **Nav2 自主导航**。

当前已实现：小车底盘物理仿真、LiDAR/IMU 传感器仿真、ros_gz_bridge 话题桥接、FAST-LIO 实时建图、关节状态发布、**Nav2 导航堆栈完整配置与启动脚本**、PCD→PGM 地图转换工具、/odom→/tf 中继节点。**当前阶段 Nav2 参数已全面调优（AMCL 粒子滤波、DWB 局部规划器、控制器超时），进入端到端仿真验证阶段。**

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
| Mali-G610 GPU 驱动不稳定 | Ogre2 渲染 segfault (exit -11) | `LIBGL_ALWAYS_SOFTWARE=1` 强制软件渲染；世界文件改用 `ogre` 引擎 |
| 8 核 ARM 多线程竞争 | colcon 编译锁死系统 | `export MAKEFLAGS="-j8"` 硬限 8 线程 |
| 有限的内存带宽 | 高频率传感器 RELIABLE QoS 导致 DDS 溢出 | LiDAR/IMU 强制 `BEST_EFFORT` QoS |
| Heightmap shader 不兼容 | `<heightmap>` 导致 Ogre shader 崩溃 | 禁止使用 heightmap，改用 `<box>` |

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
| **导航堆栈** | Nav2 (nav2_bringup) | Humble 官方 |
| **URDF 解析** | robot_state_publisher + xacro | Humble 官方 |
| **可视化** | RViz2 + nav2_rviz_plugins | Humble 官方 |
| **点云转激光** | pointcloud_to_laserscan | Humble 官方 |
| **C++ 标准** | C++17, aarch64 NEON 优化 | `-march=armv8-a+crypto -mcpu=cortex-a76` |

---

## 四、系统架构

### 4.1 数据流架构

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                          Ignition Fortress 仿真世界                            │
│                                                                              │
│  ┌──────────────────────────────────────┐                                    │
│  │   World: outdoor_flat_features       │  (10m×10m 封闭房间 + 3 柱子障碍物)   │
│  │   · 地面 / 四面围墙 / 天花板          │                                    │
│  │   · 3 个 box 柱子障碍物               │                                    │
│  │   · 渲染引擎: ogre (非 ogre2, ARM64) │                                    │
│  └──────────────┬───────────────────────┘                                    │
│                 │                                                            │
│  ┌──────────────▼───────────────────────┐                                    │
│  │   Robot: outdoor_bot (4WD)           │                                    │
│  │  ┌────────────────────────────────┐  │                                    │
│  │  │ DiffDrivePlugin (skid)         │  │  ← /model/outdoor_bot/cmd_vel     │
│  │  │  4× continuous joints          │  │                                    │
│  │  │  wheel_sep: 0.58m (修正自转)         │  │                                    │
│  │  │  wheel_radius: 0.08m           │  │                                    │
│  │  │  odom → /odom @20Hz            │  │                                    │
│  │  │  TF → /model/outdoor_bot/tf    │  │  (odom→body TF 发布)               │
│  │  └────────────────────────────────┘  │                                    │
│  │  ┌────────────────────────────────┐  │                                    │
│  │  │ LiDAR (gpu_lidar→cpu ray)     │  │  → /lidar/points (10Hz)           │
│  │  │  32线, 360°H, 59°V (-7°~+52°) │  │                                    │
│  │  │  900 samples/ring             │  │                                    │
│  │  │  range: 0.1-40m              │  │                                    │
│  │  └────────────────────────────────┘  │                                    │
│  │  ┌────────────────────────────────┐  │                                    │
│  │  │ IMU (200Hz, 6-axis Gaussian   │  │  → /imu/data (200Hz)              │
│  │  │       noise model)            │  │                                    │
│  │  └────────────────────────────────┘  │                                    │
│  │  ┌────────────────────────────────┐  │                                    │
│  │  │ JointStatePublisher(30Hz)     │  │  → joint_state                     │
│  │  └────────────────────────────────┘  │                                    │
│  └──────────────────────────────────────┘                                    │
└──────────────────────────────────┬───────────────────────────────────────────┘
                                   │
                                   │ ros_gz_bridge parameter_bridge
                                   ▼
┌──────────────────────────────────────────────────────────────────────────────┐
│                              ROS 2 层                                         │
│                                                                               │
│  /clock                          → rosgraph_msgs/Clock                       │
│  /lidar/points/points            → sensor_msgs/PointCloud2 (→ /velodyne_points) │
│  /imu/data                       → sensor_msgs/Imu                           │
│  /odom                           → nav_msgs/Odometry (DiffDrive 里程计)       │
│  /model/outdoor_bot/tf           → tf2_msgs/TFMessage   (remap → /tf)       │
│  /world/.../joint_state          → sensor_msgs/JointState (→ /world_joint_states)│
│  /cmd_vel (ROS)                  → /model/outdoor_bot/cmd_vel (Ignition)     │
│                                                                               │
│  ┌──────────────────────────────────────────────────────────────────────┐    │
│  │  odom_to_tf.py (scripts/)                                              │    │
│  │  订阅 /odom → 重新广播为 /tf (odom→body)                               │    │
│  │  ⚠️ 备选方案：当桥接的 /tf 不更新时使用                                  │    │
│  └──────────────────────────────────────────────────────────────────────┘    │
│                                                                               │
│  ┌──────────────────────────────────────────────────────────────────────┐    │
│  │                   FAST-LIO SLAM 节点                                  │    │
│  │  /velodyne_points + /imu/data → odometry → /Odometry                │    │
│  │                                     TF: map ← odom                   │    │
│  │                                     → /fastlio_mapping/global_map    │    │
│  └──────────────────────────────────────────────────────────────────────┘    │
│                                                                               │
│  ┌──────────────────────────────────────────────────────────────────────┐    │
│  │              robot_state_publisher                                   │    │
│  │  URDF/xacro → /tf_static (body, lidar_link, imu_link, 4 wheels)     │    │
│  └──────────────────────────────────────────────────────────────────────┘    │
│                                                                               │
│  ┌──────────────────────────────────────────────────────────────────────┐    │
│  │              Nav2 导航堆栈 (nav2_sim_launch.py)                        │    │
│  │                                                                        │    │
│  │  pointcloud_to_laserscan_node                                          │    │
│  │    /velodyne_points → /scan (LaserScan, 360°, 10Hz)                   │    │
│  │                                                                        │    │
│  │  pcd_to_pointcloud (pcl_ros)                                           │    │
│  │    离线 PCD 文件 → /pcl_ros/pcd/points (PointCloud2, 5s间隔)           │    │
│  │                                                                        │    │
│  │  Nav2 Bringup (map_server, AMCL, planner, controller, BT)              │    │
│  │    ├── map_server: 加载 fastlio_map.yaml/pgm → /map                    │    │
│  │    ├── AMCL: /scan + /tf → 粒子滤波定位 → map→odom TF                  │    │
│  │    ├── planner_server: NavFn 全局规划                                   │    │
│  │    ├── controller_server: DWB 局部规划 (滑移转向适配)                   │    │
│  │    ├── behavior_server: spin/backup/wait 行为                          │    │
│  │    └── bt_navigator: 导航行为树                                        │    │
│  │                                                                        │    │
│  │  RViz2 + Navigation 2 Panel                                            │    │
│  │    2D Pose Estimate / 2D Nav Goal 交互                                 │    │
│  └──────────────────────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────────────────────┘
```

### 4.2 TF 树

**当前结构（使用 FAST-LIO 建图时）：**
```
map
  └── odom (由 FAST-LIO /odom 里程计提供)
        └── body (由 robot_state_publisher /tf_static 发布)
              ├── lidar_link   (fixed, z=0.1)
              ├── imu_link     (fixed, z=0.1)
              ├── left_front_wheel_joint
              ├── left_rear_wheel_joint
              ├── right_front_wheel_joint
              └── right_rear_wheel_joint
```

**Nav2 模式下（AMCL 接管全局定位）：**
```
map
  └── odom (由 AMCL 粒子滤波发布)
        └── body (由 robot_state_publisher /tf_static 发布)
              └── ...同上...
```

**重要说明：**
- `map → odom` 在 FAST-LIO 模式下由 FAST-LIO 直接发布
- `map → odom` 在 Nav2 模式下由 AMCL 发布（接管全局定位）
- `odom → body` 在 Ignition 侧由 DiffDrive 插件发布到 `/model/outdoor_bot/tf`，通过 ros_gz_bridge 桥接到 ROS 2 `/tf`
- 如果桥接的 `/tf` 不更新，备用方案是 `odom_to_tf.py` 脚本（订阅 `/odom` 重新广播为 `/tf`）
- **Nav2 AMCL 需要 `odom` → `body` 链条完整才能正常工作**

### 4.3 机器人参数

| 参数 | 值 |
|------|-----|
| 底盘尺寸 | 0.4m × 0.3m × 0.15m |
| 底盘质量 | 10.0 kg |
| 轮距 (wheel_separation) | 0.58 m (自转误差修正后) |
| 轮半径 (wheel_radius) | 0.08 m |
| 驱动方式 | 四轮差速滑移转向 (skid-steer) |
| 机器人坐标系 frame | `body` (非 `base_link`/`base_footprint`) |
| 最大速度 (仿真) | 取决于 /cmd_vel 输入 |

---

## 五、当前桥接话题清单 (ros_gz_bridge)

**配置位置：** `src/outdoor_sim/launch/sim_launch.py`

| Ignition 话题 | 方向 | ROS 话题 (remap 后) | 消息类型 | 用途 |
|-------------|------|-------------------|---------|------|
| `/clock` | Ignition → ROS | `/clock` | `rosgraph_msgs/Clock` | 仿真时间同步 |
| `/lidar/points/points` | Ignition → ROS | `/velodyne_points` | `sensor_msgs/PointCloud2` | SLAM + Nav2 输入 |
| `/imu/data` | Ignition → ROS | `/imu/data` | `sensor_msgs/Imu` | SLAM 输入 |
| `/odom` | Ignition → ROS | `/odom` | `nav_msgs/Odometry` | 里程计 (DiffDrive) |
| `/model/outdoor_bot/tf` | Ignition → ROS | `/tf` | `tf2_msgs/TFMessage` | odom→body TF |
| `/world/.../joint_state` | Ignition → ROS | `/joint_states` | `sensor_msgs/JointState` | RViz 车轮转动 (robot_state_publisher 可读取) |
| `/model/outdoor_bot/cmd_vel` | ROS → Ignition | `/cmd_vel` | `geometry_msgs/Twist` | 速度控制 |

**注意：** 关节状态话题 remap 到 `/joint_states`，让 robot_state_publisher 可以读取轮子关节角度，在 RViz2 中渲染轮子转动。

---

## 六、FAST-LIO 配置要点

配置文件：`src/FAST_LIO_ROS2/config/mid360_sim.yaml`

| 参数 | 值 | 说明 |
|------|-----|------|
| `lidar_type` | 2 | Velodyne/PointCloud2 输入 |
| `scan_line` | 32 | 匹配 32 线雷达（xacro 已设为 32） |
| `point_filter_num` | 3 | 每 3 个点采 1 个（降采样） |
| `filter_size_surf` | 0.15 | 面特征滤波尺寸 |
| `filter_size_map` | 0.15 | 地图体素滤波尺寸 |
| `extrinsic_T` | [0, 0, 0.03] | LiDAR→IMU 平移外参 |
| `extrinsic_R` | I₃ | 旋转为 Identity（已标定） |
| `extrinsic_est_en` | false | 关闭在线估计 |
| `time_sync_en` | false | 仿真时钟完美对齐 |
| `con_est_en` | false | 仿真无运动畸变 |
| `max_iteration` | 3 | 迭代次数（兼顾精度与性能）|

---

## 七、Nav2 集成现状

### 7.1 已完成的工作

Nav2 集成已进入参数调优完成阶段，以下组件已就绪：

| 组件 | 文件 | 状态 |
|------|------|------|
| **Nav2 参数配置** | `config/nav2_params.yaml` | ✅ 已完成 |
| **Nav2 启动脚本** | `launch/nav2_sim_launch.py` | ✅ 已编写 |
| **RViz Nav2 面板** | `config/nav2_3d_view.rviz` | ✅ 已配置 |
| **PCD→PGM 转换工具** | `pcd_to_pgm.py` (项目根目录) | ✅ 已编写 |
| **/odom→/tf 中继** | `scripts/odom_to_tf.py` | ✅ 已安装（含 reverse_yaw SE(2) 镜像修复） |
| **全局时钟滤波器** | `scripts/clock_filter.py` | ✅ 已安装（支持仿真重启时间回跳） |
| **代价地图膨胀层** | nav2_params.yaml 中配置 | ✅ 已配置 |
| **PointCloud→LaserScan** | nav2_sim_launch.py 中启动 | ✅ 已集成 |
| **AMCL 粒子滤波** | nav2_params.yaml 中配置 | ✅ 已调优（auto-init + 高噪声里程计模型） |
| **DWB 局部规划器** | nav2_params.yaml 中配置 | ✅ 滑移转向适配（含终点自转优化） |
| **NavFn 全局规划器** | nav2_params.yaml 中配置 | ✅ 已配置 |
| **AMCL 自动初始化** | nav2_params.yaml | ✅ set_initial_pose=True, initial_pose=[0,0,0.26] |
| **DWB 终点超时修复** | nav2_params.yaml | ✅ slowing_factor=2.5, min_speed_theta=0.25, yaw_tolerance=0.30 |
| **控制器耐心窗口** | nav2_params.yaml | ✅ movement_time_allowance=15s |
| **桥接启动加固** | launch/sim_launch.py | ✅ use_sim_time 齐全 + 日志过滤

### 7.2 Nav2 参数配置详解 (`config/nav2_params.yaml`)

#### 全局定位 — AMCL

采用 **选项 2**（AMCL 粒子滤波），关键参数：

| 参数 | 值 | 说明 |
|------|-----|------|
| `global_frame_id` | `map` | 全局坐标系 |
| `odom_frame_id` | `odom` | 里程计坐标系 |
| `base_frame_id` | **`body`** | ⚠️ 匹配 URDF 根 link 名 |
| `set_initial_pose` | `True` | 开机自动发布 map→odom |
| `initial_pose` | `[0, 0, 0.26]` | 初始偏角 +15°，补偿建图倾角 |
| `initial_cov_aa` | 0.15 | 初始角度协方差（~20° 搜索空间）|
| `max_particles` | 2000 | 降低粒子数以节省 RK3588 CPU（默认 5000） |
| `min_particles` | 500 | 最少粒子数 |
| `update_min_a` | **0.12** | 角度更新阈值 ~6.8° |
| `update_min_d` | **0.10** | 位移更新阈值 10 cm |
| `max_beams` | **120** | 参与匹配光束翻倍，转弯瞬间锁死墙壁 |
| `alpha1` | **1.5** | 旋转→旋转噪声（"自转时里程计完全不可信"）|
| `alpha2` | 0.4 | 直行→旋转噪声 |
| `alpha3` | 0.4 | 直行→位移噪声 |
| `alpha4` | **1.5** | 旋转→位移噪声（"自转时完全依赖雷达"）|
| `laser_model_type` | `likelihood_field` | 激光观测模型（比 beam 模型快） |
| `scan_topic` | `/scan` | 激光输入（来自 pointcloud_to_laserscan） |

#### 局部规划 — DWB (滑移转向适配)

| 参数 | 值 | 说明 |
|------|-----|------|
| `max_vel_x` | 0.5 m/s | 最高线速度 |
| `max_vel_y` | **0.0** | 差速底盘无 y 向速度 |
| `max_vel_theta` | 1.0 rad/s | 最高角速度 |
| `min_speed_theta` | **0.25** | 原地自旋最小转速（提供足够扭矩克服滑移摩擦）|
| `yaw_goal_tolerance` | **0.30** | 终点角度容差放宽~17°，容忍滑移误差 |
| `vx_samples` | 20 | 线速度采样 |
| `vtheta_samples` | 20 | 角速度采样 |
| `sim_time` | 1.7 s | 轨迹模拟时长 |

**Critics 权重（经过调优）：**

| Critic | 权重 | 作用 |
|--------|------|------|
| `BaseObstacle` | 0.02 | 障碍物避碰（权重低，因为 DWB 在滑移转向下转弯半径大） |
| `PathAlign` | 10.0 | 路径对齐 |
| `GoalAlign` | 16.0 | 目标对齐 |
| `PathDist` | 32.0 | 路径距离 |
| `GoalDist` | 24.0 | 目标距离 |
| `RotateToGoal` | 32.0 | 终点旋转 |
| `RotateToGoal.slowing_factor` | **2.5** | 接近终点时减速阻尼减半，防卡死 |

**轮廓检查器超时：**

| 参数 | 值 | 说明 |
|------|-----|------|
| `movement_time_allowance` | **15.0 s** | 终点调向超时窗口延长，避免 `Controller patience exceeded` abort |

#### 代价地图

| 层 | 全局地图 | 局部地图 |
|------|---------|---------|
| **尺寸** | 10×10m (全地图) | 3×3m (rolling window) |
| **分辨率** | 0.05 m/px | 0.05 m/px |
| **更新频率** | 1.0 Hz | 5.0 Hz |
| **插件** | `static_layer` + `inflation_layer` | `obstacle_layer` + `inflation_layer` |
| **膨胀半径** | 0.35 m | 0.25 m |
| **Footprint** | `[[-0.2,-0.15], [-0.2,0.15], [0.2,0.15], [0.2,-0.15]]` | 同左 |

#### 全局规划 — NavFn

| 参数 | 值 |
|------|-----|
| `plugin` | `nav2_navfn_planner/NavfnPlanner` |
| `tolerance` | 0.5 m |
| `use_astar` | false (使用 NavFn，比 A* 快) |
| `allow_unknown` | true |

### 7.3 PCD → PGM 地图转换工具 (`pcd_to_pgm.py`)

**位置：** 项目根目录 `/home/orangepi/ros2_ws/pcd_to_pgm.py`

将 FAST-LIO 保存的 `.pcd` 3D 点云地图转换为 Nav2 可用的 2D 栅格地图：

```
my_3d_map.pcd (FAST-LIO 保存)
    ↓ python3 pcd_to_pgm.py
fastlio_map.pgm + fastlio_map.yaml (Nav2 map_server 加载)
```

**转换参数：**
- `Z_MIN` = 0.1 m（滤除地面）
- `Z_MAX` = 1.5 m（滤除天花板/噪点）
- `RESOLUTION` = 0.05 m/px
- 障碍物膨胀半径 = 0.25 m（机器人半径）
- 投影方式：高度切片 + floodFill 空闲区域标注

**输出文件示例 (`fastlio_map.yaml`)：**
```yaml
image: fastlio_map.pgm
mode: trinary
resolution: 0.05
origin: [x_min, y_max, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

### 7.4 Odom→TF 中继 (`scripts/odom_to_tf.py`)

**背景：** Ignition DiffDrive 通过 `<update_odom_to_tf>true</update_odom_to_tf>` 在 Ignition 内发布 Pose_V 到 `/tf` 话题，经 ros_gz_bridge 桥接到 ROS 2。实测中发现桥接后的 TF 可能不更新（卡在初始化位姿）。

**解决方案：** `odom_to_tf.py` 直接订阅 `/odom`（nav_msgs/Odometry，以 20Hz 正常更新），将位姿重新广播为 ROS 2 TF（`odom` → `body`），绕过桥接层卡顿。

**SE(2) 镜像修复：** 内置 `REVERSE_YAW` 开关（当前 `False`）。若 DiffDrive 里程计自转方向反向，设为 `True` 后同时对四元数取共轭且翻转 Y 轴位移，保持右手坐标系一致性，解决 DWB `No valid trajectories` 问题。

**时间戳鲁棒性：** 过滤 >1s 回退视为仿真重启并重新同步，防止 TF 树死锁。

**启动方式：** 已在 sim_launch.py 中自动启动，`use_sim_time=True`。也可单独运行：
```bash
ros2 run outdoor_sim odom_to_tf.py
```

### 7.5 Nav2 启动流程

```bash
# 终端 1：启动仿真（Headless）
export LIBGL_ALWAYS_SOFTWARE=1
xvfb-run -a ros2 launch outdoor_sim sim_launch.py

# 等待仿真稳定后（5-10秒）
# 终端 2：启动 FAST-LIO SLAM（如需建图）
ros2 launch fast_lio mapping.launch.py config:=mid360_sim.yaml

# 终端 3：（可选）控制小车建图
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.2}}"

# 建图完成后保存 PCD
ros2 run fast_lio save_map --save_path /home/orangepi/ros2_ws/my_3d_map.pcd

# 转换 PCD → PGM
python3 ~/ros2_ws/pcd_to_pgm.py

# 终端 4：启动 Nav2 导航（需先关闭 FAST-LIO 和建图 cmd_vel）
ros2 launch outdoor_sim nav2_sim_launch.py
```

### 7.6 Nav2 启动文件详解 (`launch/nav2_sim_launch.py`)

启动文件包含三个主要部分：

1. **pointcloud_to_laserscan** 节点
   - 输入：`/velodyne_points` (PointCloud2)
   - 输出：`/scan` (LaserScan, 360°, 10Hz)
   - Z轴滤波：0.1~1.0m（滤除地面和天花板）
   - 角度分辨率：0.0087 rad (~5°)

2. **pcd_publisher** (pcl_ros `pcd_to_pointcloud`)
   - 输入：`~/ros2_ws/my_3d_map.pcd`（离线文件）
   - 输出：`/pcl_ros/pcd/points` (PointCloud2)
   - 发布间隔：5000ms（节省 CPU）
   - QoS：`best_effort`

3. **Nav2 bringup** (nav2_bringup `bringup_launch.py`)
   - `slam=False`（使用已有地图）
   - `map=fastlio_map.yaml`
   - `params_file=nav2_params.yaml`

4. **Nav2 RViz2** (nav2_bringup `rviz_launch.py`)
   - 带 Navigation 2 面板（2D Pose Estimate + 2D Nav Goal）

### 7.7 ARM64/RK3588 特定约束

```
┌─────────────────────────────────────────────────────────────────────────┐
│                     🚨 ARM64 部署警告 🚨                                 │
│                                                                          │
│  1. Nav2 参数适配                                                       │
│     · robot_base_frame = "body"（不是 base_link，已全部对齐）            │
│     · AMCL 粒子数降为 2000（默认 5000 对 RK3588 太高）                   │
│     · costmap 更新频率降低（全局 1Hz，局部 5Hz）                         │
│                                                                          │
│  2. QoS 一致性                                                          │
│     · FAST-LIO 使用 BEST_EFFORT 接收 LiDAR/IMU                          │
│     · Nav2 costmap 的 obstacle_layer 默认订阅 /scan（可配置 QoS）         │
│     · pointcloud_to_laserscan 使用 use_sim_time=true                     │
│                                                                          │
│  3. 仿真时间 vs 真实时间                                                 │
│     · 所有 Nav2 参数中 use_sim_time=True                                 │
│     · BT 超时参数已相应调整                                              │
│                                                                          │
│  4. 点云数据量                                                          │
│     · 32 线 × 900 点每线 = 28,800 点/帧，对 RK3588 可接受               │
│     · pointcloud_to_laserscan 进一步降采样为 LaserScan                   │
│     · PCD 发布周期设为 5s，避免 CPU 过载                                 │
│                                                                          │
│  5. 内存预算                                                             │
│     · Ignition Fortress (server only): ~600MB                           │
│     · FAST-LIO SLAM: ~200-400MB                                          │
│     · Nav2 全套: ~200-300MB                                              │
│     · 总计约 1.2-1.5 GB，16GB 绰绰有余                                   │
│                                                                          │
│  6. 渲染引擎                                                             │
│     · 世界文件使用 <render_engine>ogre</render_engine> 而非 ogre2        │
│     · ogre2 在 Mali-G610 上 segfault                                     │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 八、当前项目文件结构

```
~/ros2_ws/
├── src/
│   ├── outdoor_sim/                          # 仿真包 (本包)
│   │   ├── worlds/
│   │   │   └── grass_terrain.world           # SDF 1.8 世界 (10m×10m 房间)
│   │   ├── urdf/
│   │   │   └── robot_sensors.xacro           # 机器人 URDF (32线 LiDAR + 200Hz IMU)
│   │   ├── launch/
│   │   │   ├── sim_launch.py                 # 主仿真启动文件
│   │   │   └── nav2_sim_launch.py            # Nav2 导航启动文件 ★新增
│   │   ├── config/
│   │   │   ├── nav2_params.yaml              # Nav2 完整参数配置 ★新增
│   │   │   └── nav2_3d_view.rviz             # Nav2 RViz 配置 ★新增
│   │   ├── rviz/
│   │   │   └── nav2_default.rviz             # 默认 RViz 配置 ★新增
│   │   ├── scripts/
│   │   │   └── odom_to_tf.py                 # /odom→/tf 中继节点 ★新增
│   │   ├── CMakeLists.txt                    # 安装配置 (含新目录)
│   │   └── package.xml                       # 依赖声明
│   │
│   └── FAST_LIO_ROS2/                        # SLAM 建图包
│       ├── config/mid360_sim.yaml            # 仿真专用配置
│       ├── launch/                           # FAST-LIO 启动文件
│       ├── src/                              # C++ 源码 (C++17, NEON)
│       ├── include/                          # 头文件
│       └── package.xml
│
├── pcd_to_pgm.py                             # PCD → PGM 地图转换工具 ★新增
├── CLAUDE.md                                 # 项目规则文档
├── project_overview_for_gemini.md            ← 本文件
└── (build/, install/, log/ — 均为 gitignored)
```

---

## 九、启动命令速查

```bash
# ── 0. 环境变量（每次新终端都执行） ──────────────────────────
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export MAKEFLAGS="-j8"
source ~/ros2_ws/install/setup.bash

# ── 1. 启动仿真（Headless） ────────────────────────────────
export LIBGL_ALWAYS_SOFTWARE=1
xvfb-run -a ros2 launch outdoor_sim sim_launch.py

# ── 2. 启动 FAST-LIO SLAM 建图 ─────────────────────────────
ros2 launch fast_lio mapping.launch.py config:=mid360_sim.yaml

# ── 3. 控制小车建图 ────────────────────────────────────────
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.2}}"

# ── 4. 保存 PCD 地图 ───────────────────────────────────────
ros2 run fast_lio save_map --save_path ~/ros2_ws/my_3d_map.pcd

# ── 5. PCD → PGM 转换 ─────────────────────────────────────
python3 ~/ros2_ws/pcd_to_pgm.py

# ── 6. 启动 Nav2 导航（需在仿真运行的基础上） ──────────────
# 先关闭 FAST-LIO（Ctrl+C），再启动 Nav2
ros2 launch outdoor_sim nav2_sim_launch.py

# ── 7. 检查话题 ────────────────────────────────────────────
ros2 topic list
ros2 topic hz /velodyne_points
ros2 topic hz /imu/data
ros2 topic hz /odom
ros2 topic echo /scan  # Nav2 启动后检查

# ── 8. 查看 TF 树 ──────────────────────────────────────────
ros2 run tf2_tools view_frames

# ── 9. 单独运行 odom→tf 中继 ───────────────────────────────
ros2 run outdoor_sim odom_to_tf.py
```

---

## 十、当前已知问题

| # | 问题 | 状态 | 说明 |
|---|------|------|------|
| 1 | LiDAR 从 64 线降至 32 线 | ✅ 已优化 | xacro 中垂直 samples=32，降低 CPU 防止掉帧 |
| 2 | 车轮 3D 模型在 RViz 不可见 | ✅ 已修复 | JointStatePublisher + 桥接到 `/joint_states` |
| 3 | 室内 10m×10m 封闭环境 | ✅ 已实现 | 四面墙 + 天花板 + 3 柱子 |
| 4 | FAST-LIO 外参标定 | ✅ 已修正 | extrinsic_T=[0,0,0.03] |
| 5 | 没有 2D 地图供 Nav2 | ✅ 工具已就绪 | `pcd_to_pgm.py` 可将 PCD 转 OccupancyGrid |
| 6 | 没有 /scan 话题 | ✅ 已解决 | nav2_sim_launch.py 中已集成 pointcloud_to_laserscan |
| 7 | 轮式里程计漂移大 | ✅ 已抑制 | AMCL alpha1=1.5/alpha4=1.5 强制信任雷达；wheel_sep 0.36→0.58 修正自转误差 |
| 8 | 没有回环检测 | ⚠️ 已知 | FAST-LIO 无回环，长时间运行地图会漂移 |
| 9 | Nav2 参数配置 | ✅ 已调优 | AMCL、DWB、控制器超时均已调优（含 auto-init、抗打滑、终点超时修复）|
| 10 | **TF 桥接不更新** | ✅ 已绕过 | `odom_to_tf.py` 直接订阅 /odom 广播 TF；含 reverse_yaw 镜像修复 |
| 11 | 机器人 frame id 对齐 | ✅ 已验证 | Nav2 使用 `body`，所有配置已对齐 |
| 12 | **PCD→PGM 质量** | ⚠️ 待验证 | 需要实际运行建图后验证转换效果和地图可用性 |
| 13 | **Nav2 端到端待验证** | 🔴 待测试 | 参数已全面调优，需在仿真中运行完整导航流程验证 |
| 14 | 渲染引擎降级 | ✅ 已适配 | 世界文件使用 `ogre` 而非 `ogre2` 避免 ARM64 segfault |
| 15 | LiDAR 垂直 FOV 非对称 | ⚠️ 已知 | xacro 中 min_angle=-7°, max_angle=+52°，非对称向上倾斜 |
| 16 | **自转方向一致性** | ✅ 已修复 | REVERSE_YAW=False（物理正确）+ SE(2) 同步 Y 轴镜像 |
| 17 | **仿真重启死锁** | ✅ 已修复 | clock_filter + odom_to_tf 均支持 >1s 回跳重新同步 |
| 18 | **终点超时 abort** | ✅ 已修复 | slowing_factor=2.5, yaw_tolerance=0.30, movement_time_allowance=15s |

---

> **给 Gemini 架构师的核心问题：**
>
> Nav2 参数已全面调优（AMCL 噪声模型、DWB 终点行为、控制器超时），当前最大不确定因素是 **端到端仿真验证**。请重点关注：
> 1. **导航成功率** — 在 10×10m 三柱子房间中，给定 2D Nav Goal 后能否稳定规划并到达？DWB 在大转弯半径滑移下是否仍有 `No valid trajectories`？
> 2. **PCD→PGM 地图质量** — FAST-LIO 3D 地图投影为 2D 后，柱子/墙壁轮廓是否清晰？是否需要对 `pcd_to_pgm.py` 调整 Z 轴切片参数？
> 3. **TF 时间戳稳定性** — `clock_filter.py` + `odom_to_tf.py` 在仿真重启后是否正常重新同步？TF 树是否有回跳？
> 4. **ARM64 性能** — 同时运行 Ignition (ogre) + Nav2 全套，RK3588 各核心负载是否均衡？AMCL `max_particles=2000` + `max_beams=120` 是否仍有余量？
> 5. **地图初始偏角校准** — `initial_pose=[0,0,0.26]` (+15°) 是否足够精确？是否需要进一步微调？
