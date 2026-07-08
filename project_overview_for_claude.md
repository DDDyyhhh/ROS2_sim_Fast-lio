# 🚜 自动割草机 — RK3588 实车开发指南

> 面向 Claude 的项目总览文档  
> 用途：在 RK3588 (Orange Pi 5 Plus) 上部署并继续开发

---

## 一、项目概述

自动割草机，支持用户在手机/电脑地图上画多边形框定区域 → 牛耕式全覆盖路径规划 → 沿路径执行割草。

### 三层系统架构

```
PC (WSL2)                    RK3588 (Orange Pi 5 Plus)      STM32F407
┌─────────────────┐          ┌──────────────────────┐      ┌──────────────┐
│ 代码编辑 + 仿真  │          │ 上位机: ROS2 Humble  │      │ 下位机:       │
│ ROS2 Humble     │          │                      │      │ 电机控制      │
│ Ignition        │  git     │ • FAST-LIO SLAM      │ CAN  │ 刀盘控制      │
│ Fortress        │ ────────→│ • 全覆盖路径规划      │────→│ 安全急停      │
│ RViz2 可视化    │          │ • Nav2 导航           │      │ 编码器反馈    │
└─────────────────┘          │ • RTK GPS 融合        │      └──────────────┘
                              │ • 路径执行管理        │
                              └──────────────────────┘
```

### 传感器

| 传感器 | 型号 | 接口 | 用途 |
|--------|------|------|------|
| LiDAR | DJI Mid-360 | Ethernet | SLAM + 避障 |
| RTK GPS | BT-982G1 | USB (串口) | 绝对定位 + 区域框定 |
| IMU | BMI088 (Mid-360内置) | — | 惯导融合 |

---

## 二、已完成的工作 (Phase 0+1)

以下内容已在 WSL2 仿真中验证通过，代码已提交到 Git：

### 仿真环境
- **50m×50m 室外草坪** — 含树木/花坛/石头/喷灌头障碍物
- **随机障碍物生成脚本** — `random_obstacles.py --count 10 --append`
- **Nav2 双模式参数** — 纯导航 (AMCL) + SLAM 导航 (FAST-LIO)

### mower_coverage 包（全覆盖路径规划）

| 模块 | 文件 | 功能 |
|------|------|------|
| 牛耕式规划器 | `boustrophedon_planner.py` | 多边形→平行条带→障碍物跳过→转弯连接 |
| 区域定义 | `area_definer.py` | 服务接口 + YAML 持久化 + RViz 可视化 |
| 路径执行器 | `path_executor.py` | direct/Nav2 双模式、断点续割、边界保护 |
| 覆盖率监控 | `coverage_monitor.py` | 栅格热力图 + 实时 m² 统计 |
| 整合演示 | `coverage_demo.py` | 一键 `/coverage/plan_and_start` |

### 修复的问题
- ✅ `ColorRGBA` 未导入 → path_executor crash
- ✅ 越过目标点不停 → `_prev_min_dist` 越点检测
- ✅ `type="ambient"` → `<scene><ambient>` SDF 兼容
- ✅ 世界名硬编码 → `outdoor_grassland_50x50`
- ✅ 服务类型错误 → area_definer 用 Trigger 替代
- ✅ `nav2_msgs` 条件导入 → direct 模式不依赖

---

## 三、RK3588 开发环境搭建

### 系统要求

```bash
# 已在 RK3588 上测试通过
cat /etc/os-release        # Ubuntu 22.04 LTS (Jammy)
uname -m                   # aarch64 (ARM64)
```

### 安装 ROS2 Humble

```bash
# 如果还未安装
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge
```

### 安装 CycloneDDS（RK3588 必选）

FastRTPS 在 RK3588 上不稳定，必须用 CycloneDDS：

```bash
sudo apt install ros-humble-cyclonedds
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 添加到 ~/.bashrc
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
```

### 克隆并编译

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@github.com:DDDyyhhh/ROS2_sim_Fast-lio.git ROS2_sim_Fast-lio

# 安装依赖
sudo apt install -y ignition-fortress
pip install shapely

# 编译
cd ~/ros2_ws
export MAKEFLAGS="-j8"
colcon build --symlink-install
```

> ⚠️ RK3588 有 8 核但内存带宽有限，`-j8` 防止锁死。

### 修复 libexec 问题

```bash
mkdir -p install/mower_coverage/lib/mower_coverage
for exe in area_definer boustrophedon_planner coverage_demo coverage_monitor path_executor; do
  ln -sf ../../bin/$exe install/mower_coverage/lib/mower_coverage/
done
```

---

## 四、Phase 2：RTK + LiDAR 融合（硬件到货后执行）

### 4.1 Mid-360 LiDAR 驱动

Mid-360 使用 Livox SDK2（非 SDK1），需要 `livox_ros_driver2`：

```bash
cd ~/ros2_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

# Mid-360 需要配置 msg_MID360.py 或对应的 JSON 配置
# 关键：frame_id 需要对齐（设为 lidar_link）
```

**FAST-LIO 配置调整（实车 vs 仿真）：**

| 参数 | 仿真值 | 实车建议 | 说明 |
|------|--------|---------|------|
| `lidar_type` | 2 (PointCloud2) | 1 (Livox) | 实车用 Livox 原生格式，仿真用模拟点云 |
| `point_filter_num` | 3 | 1 | 实车点云稀疏，全量使用 |
| `time_sync_en` | false | true | 实车需要时间同步 |
| `extrinsic_est_en` | false | true → false | 先在线标定，收敛后固定 |
| `scan_line` | 32 | 6 | Mid-360 只有 6 线（非重复扫描） |

**关键：Mid-360 是固态 LiDAR（非重复扫描模式），FAST-LIO 的配置与机械式 LiDAR 不同。**

### 4.2 BT-982G1 RTK GPS 驱动

```bash
sudo apt install ros-humble-nmea-navsat-driver
```

**NTRIP 客户端（获取网络 RTK 差分数据）：**

BT-982G1 通过 NTRIP 获取 RTCM 差分修正，有两种方案：

**方案 A：ntrip_ros 包**
```bash
cd ~/ros2_ws/src
git clone https://github.com/LORD-MicroStrain/ntrip_ros.git
# 配置账号：ntrip_host, port, mountpoint, user, password
```

**方案 B：自写 Python NTRIP 客户端**
```python
# 核心逻辑：socket 连接 caster，发送 NTRIP 请求
# 将 RTCM 数据通过串口转发给 BT-982G1
# 或者直接在 ROS 节点内解析，发布到 /rtcm
```

### 4.3 robot_localization EKF 融合

```bash
sudo apt install ros-humble-robot-localization
```

**融合架构：**

```
RTK GPS ─→ nmea_navsat_driver ─→ /gps/fix (nav_msgs/Odometry)
BT-982G1                                  |
      + → navsat_transform_node ─→ /odom/gps (转换到 UTM)
      |
Mid-360 ─→ FAST-LIO ─→ /odom (SLAM 里程计)
      |
IMU ─────────────────→ /imu/data
      |
      + → EKF (ekf_localization_node) ─→ /odometry/filtered
```

**navsat_transform_node 配置要点：**
```yaml
# robot_localization 的 navsat_transform 需要正确设置
# 否则 GPS 数据不会被正确转换到 UTM 坐标系
frequency: 30
delay: 3.0  # 等待 GPS 收敛到固定解
magnetic_declination_radians: 0.0
yaw_offset: 0.0
zero_altitude: true
publish_filtered_gps: true
use_odometry_yaw: false
wait_for_datum: true
```

### 4.4 坐标系对齐

```yaml
# 传感器外参
# LiDAR→IMU: 由 FAST-LIO 在线标定
# GPS 天线杆臂: 测量物理安装偏移 → 配置到 navsat_transform
#  
# TF 树（实车）:
# map ──(FAST-LIO)──→ odom ──(static)──→ body
#                                           ├── lidar_link
#                                           ├── imu_link
#                                           └── gps_link (GPS 天线相位中心)
```

---

## 五、Phase 3：CAN 总线 + STM32 下位机

### 5.1 CAN 硬件接口

RK3588（Orange Pi 5 Plus）有 CAN 控制器，通过 SPI 或内置 CAN：

```bash
# 检查 CAN 接口
ip link show can0
# 如果不存在，需要设备树启用
# Orange Pi 5 Plus 扩展引脚有 CAN 接口
```

### 5.2 ROS2 CAN 通信

```bash
sudo apt install ros-humble-can-msgs
```

**协议设计（建议）：**

**RK3588 → STM32（控制帧，50Hz）：**
| ID | 数据 | 类型 | 说明 |
|----|------|------|------|
| 0x100 | [vx, vz] | float32 × 2 | 线速度 + 角速度 |
| 0x101 | [blade_enable, reserved] | uint8 × 2 | 刀盘启停 |
| 0x102 | [mode, reserved] | uint8 × 2 | 工作模式(停止/作业/回泊) |

**STM32 → RK3588（状态帧，100Hz）：**
| ID | 数据 | 类型 | 说明 |
|----|------|------|------|
| 0x200 | [left_wheel, right_wheel] | int32 × 2 | 编码器位置 |
| 0x201 | [left_speed, right_speed] | float32 × 2 | 轮速 |
| 0x202 | [battery_voltage, blade_current] | float32 × 2 | 电池/刀盘电流 |
| 0x203 | [error_flags] | uint32 | 故障位(碰撞/倾覆/急停) |

### 5.3 STM32F407 固件要点

- CAN 中断接收控制指令 → PID 控制器 → PWM 输出给驱动桥
- 编码器定时器捕获 → CAN 发送回 RK3588
- 独立看门狗：100ms 内未收到 CAN 消息 → 自动停机
- 刀盘电流检测：突增 50% → 碰撞检测 → CAN 发送急停

### 5.4 安全机制

| 安全功能 | 实现位置 | 触发条件 | 后果 |
|---------|---------|---------|------|
| 电子围栏 | RK3588 (path_executor) | GPS 超出区域 | 停止 + 报警 |
| 碰撞检测 | STM32 (刀盘电流) | 电流突增 | 停止刀盘 + 后退 |
| 倾覆检测 | STM32 (IMU) | 倾角 > 45° | 急停 |
| 通信超时 | STM32 (看门狗) | 100ms 无 CAN | 自动停机 |
| 物理急停 | STM32 (GPIO) | 按钮按下 | 切断驱动电源 |

---

## 六、Phase 4：测试流程

### 6.1 测试金字塔

```
级别 1: 纯仿真 (PC)
  • ros2 launch outdoor_sim sim_launch.py
  • ros2 launch mower_coverage coverage_planning.launch.py
  • 验证路径规划 + 覆盖率 > 95%
  
级别 2: 半实物仿真 (RK3588)
  • 跑算法，不驱动电机
  • 验证 LiDAR 点云 + FAST-LIO 建图
  • 验证 RTK GPS 收敛 + NTRIP 连接
  
级别 3: 小范围场地 (50m²)
  • 安全封闭区域
  • 验证全覆盖路径执行
  • 验证安全机制（围栏/碰撞）
  
级别 4: 全尺寸草坪 (500-2000m²)
  • 实车割草测试
  • 验证长时间稳定性
  • 验证断点续割
```

### 6.2 实车启动流程

```bash
# 终端 1：启动 FAST-LIO SLAM
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/ros2_ws/install/setup.bash
ros2 launch fast_lio mapping.launch.py config:=mid360.yaml

# 终端 2：启动全覆盖规划系统
source ~/ros2_ws/install/setup.bash
ros2 launch mower_coverage coverage_planning.launch.py mode:=nav2

# 终端 3：规划+执行
source ~/ros2_ws/install/setup.bash
ros2 service call /coverage/plan_and_start std_srvs/srv/Trigger

# 终端 4：监控
ros2 topic echo /coverage/statistics
ros2 run tf2_tools view_frames
```

---

## 七、关键文件索引

| 文件 | 说明 | 开发中的重要性 |
|------|------|--------------|
| `src/outdoor_sim/launch/sim_launch.py` | 仿真启动（PC 开发用，实车不用） | 低 |
| `src/outdoor_sim/worlds/grassland_50x50.world` | 仿真世界（PC 开发用） | 低 |
| `src/mower_coverage/mower_coverage/boustrophedon_planner.py` | **核心算法：牛耕式规划** | ⭐⭐⭐ |
| `src/mower_coverage/mower_coverage/path_executor.py` | **路径执行器（需适配实车）** | ⭐⭐⭐ |
| `src/mower_coverage/mower_coverage/coverage_demo.py` | **演示整合节点** | ⭐⭐ |
| `src/mower_coverage/mower_coverage/area_definer.py` | 区域定义服务 | ⭐ |
| `src/mower_coverage/mower_coverage/coverage_monitor.py` | 覆盖率监控 | ⭐ |
| `src/mower_coverage/config/coverage_params.yaml` | **可调参数（速度/割幅等）** | ⭐⭐ |
| `src/FAST_LIO_ROS2/config/mid360.yaml` | **实车需修改 LiDAR 类型** | ⭐⭐⭐ |
| `src/outdoor_sim/config/nav2_params.yaml` | Nav2 纯导航参数（实车用） | ⭐⭐ |
| `src/outdoor_sim/config/nav2_slam_params.yaml` | Nav2 SLAM 导航参数（实车用） | ⭐⭐ |
| `OPERATION_GUIDE.md` | 仿真启动操作指南 | ⭐ |

---

## 八、已知问题和风险

| # | 问题 | 影响 | 对策 |
|---|------|------|------|
| 1 | **空旷区域 FAST-LIO 退化** | 50m×50m 仅有少量树干特征，LiDAR SLAM 可能发散 | GPS 辅助初始化 + 松耦合 EKF |
| 2 | **WSL2 仿真不稳定** | Ignition 跑几分钟闪退 | 不影响 RK3588 实车，RK3588 有硬件 GPU |
| 3 | **RTK 收敛时间** | BT-982G1 首次开机需 30-60s 收敛到固定解 | 启动延迟 60s 再开始规划 |
| 4 | **NTRIP 网络依赖** | RTK 差分需要 4G/WiFi 网络 | 增加断网保护 + 回退到单点 GPS |
| 5 | **滑移转向偏差** | 差速转弯在草地上会滑移，实际路径偏离规划 | 扩大割幅重叠率 + 定期修正 |
| 6 | **行人检测** | 目前只依赖 Nav2 局部代价地图 | 可增加 Mid-360 点云行人识别 |
| 7 | **mower_coverage libexec** | `ament_python` 不自动创建 libexec | 见 第三章 修复命令 |

---

## 九、代码修改指引

### 实车适配需修改的文件

1. **`path_executor.py`** — 实车模式下需补充 CAN 速度指令发布
2. **`coverage_params.yaml`** — 根据割幅调整 `cutting_width`
3. **`mid360.yaml`** — `lidar_type` 改为 1（Livox 格式）
4. **`sim_launch.py`** — 实车不需要启动（用 FAST-LIO 代替）

### 新增文件清单（Phase 2-3）

```
src/
├── livox_ros_driver2/          # Mid-360 驱动（从 Livox SDK 克隆）
├── ntrip_ros/                  # NTRIP 客户端（或自写）
├── mower_can_bridge/           # CAN 总线 ROS2 驱动（新建包）
│   ├── mower_can_bridge/
│   │   └── can_bridge.py       # socketcan ↔ ROS2 消息转换
│   ├── config/can_params.yaml
│   └── launch/can_bridge.launch.py
└── mower_coverage/             # 已有包，需修改
    └── mower_coverage/
        └── path_executor.py    # 增加 CAN 控制模式
```

---

## 十、与 Claude 协作建议

### 提问模板

```
在 RK3588 上部署割草机项目：
  - 当前工作：X（如：编译、测试 LiDAR、配置 RTK）
  - 遇到问题：Y（如：编译报错、点云不显示）
  - 已尝试：Z（如：已装驱动）
```

### 常用调试命令

```bash
# 检查 ROS2 环境
echo $RMW_IMPLEMENTATION
ros2 topic list

# 检查硬件
ip link show can0
ls /dev/ttyUSB*        # BT-982G1 串口
ls /dev/ttyACM*        # 可能的 GPS 串口

# 检查 CAN
candump can0

# 检查 GPS
stty -F /dev/ttyUSB0 115200
cat /dev/ttyUSB0       # 应看到 $GPGGA, $GPRMC 等 NMEA 语句

# FAST-LIO 测试
ros2 launch fast_lio mapping.launch.py config:=mid360.yaml

# 全覆盖规划
ros2 service call /coverage/plan std_srvs/srv/Trigger
``` 

---

**项目仓库：** `git@github.com:DDDyyhhh/ROS2_sim_Fast-lio.git`  
**当前分支：** `main` (Phase 0+1 已完成)  
**下一阶段：** Phase 2 (硬件到货后)
