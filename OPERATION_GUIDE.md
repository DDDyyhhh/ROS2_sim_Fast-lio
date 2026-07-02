# 🚜 自动割草机 — 仿真操作指南

## 目录
- [环境准备](#环境准备)
- [一键启动（推荐）](#一键启动推荐)
- [分步启动](#分步启动)
- [可视化说明](#可视化说明)
- [常用命令](#常用命令)
- [故障排除](#故障排除)

---

## 环境准备

### 首次安装依赖

```bash
# 安装 Ignition Fortress 仿真引擎（如未安装）
sudo apt install -y ignition-fortress

# 安装 Python 依赖
pip install shapely
```

### 编译项目

```bash
cd ~/mower_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

> ⚠️ 如果编译后 launch 找不到节点：
> ```bash
> mkdir -p install/mower_coverage/lib/mower_coverage
> for exe in area_definer boustrophedon_planner coverage_demo coverage_monitor path_executor; do
>   ln -sf ../../bin/$exe install/mower_coverage/lib/mower_coverage/
> done
> ```

---

## 一键启动（推荐）

```bash
cd ~/mower_ws
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1

# 1️⃣ 启动 50m×50m 草坪仿真（headless）
ros2 launch outdoor_sim sim_launch.py
```
> 等待约 15~20 秒，直到看到 `OK creation of entity`

**新开终端：**
```bash
cd ~/mower_ws
source install/setup.bash

# 2️⃣ 启动 RViz2 可视化
ros2 run rviz2 rviz2 -d install/outdoor_sim/share/outdoor_sim/config/coverage_viz.rviz

# 3️⃣ 发布 map→odom TF（否则 RViz2 报 "Frame map does not exist"）
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

**再开一个终端：**
```bash
cd ~/mower_ws
source install/setup.bash

# 4️⃣ 启动覆盖规划系统
ros2 launch mower_coverage coverage_planning.launch.py mode:=direct

# 5️⃣ 规划全覆盖路径
ros2 service call /coverage/plan std_srvs/srv/Trigger

# 6️⃣ 开始执行
ros2 service call /coverage/start std_srvs/srv/Trigger
```

---

## 分步启动

### 终端 1：仿真

```bash
cd ~/mower_ws
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch outdoor_sim sim_launch.py
```

启动内容：
- Ignition Gazebo 服务器（50m×50m 室外草坪）
- 机器人模型（4WD 差速底盘 + 32线 LiDAR + 200Hz IMU）
- 话题桥接（/velodyne_points, /imu/data, /odom, /cmd_vel）
- TF 中继（odom_to_tf → 发布 odom→body TF）
- 时钟滤波器

**验证：**
```bash
# 在另一个终端检查
ros2 topic list
# 应看到：/clock, /odom, /velodyne_points, /imu/data, /cmd_vel, /tf, /tf_static 等
```

### 终端 2：可视化（RViz2）

```bash
cd ~/mower_ws
source install/setup.bash
ros2 run rviz2 rviz2 -d install/outdoor_sim/share/outdoor_sim/config/coverage_viz.rviz
```

RViz2 配置包含：
| 显示层 | 话题 | 说明 |
|--------|------|------|
| Grid | — | 栅格地图 |
| TF | /tf, /tf_static | 坐标系树 |
| LiDAR PointCloud | /velodyne_points | 点云（白色） |
| Area Boundary | /coverage/area_markers | 绿色区域 |
| Coverage Path | /coverage/path_markers | 蓝色路径线+箭头 |
| Execution Status | /coverage/execution_markers | 绿色已覆盖+紫色目标点 |
| Coverage Map | /coverage/coverage_map | 覆盖热力图 |
| Robot Model | /robot_description | 小车4WD模型 |

**修复 `Frame [map] does not exist`：**
```bash
# 新开终端
cd ~/mower_ws
source install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

### 终端 3：覆盖规划系统

```bash
cd ~/mower_ws
source install/setup.bash
ros2 launch mower_coverage coverage_planning.launch.py mode:=direct
```

启动 4 个节点：
| 节点 | 功能 |
|------|------|
| area_definer | 区域定义服务 |
| coverage_demo | 路径规划演示（默认 46m×46m 区域） |
| path_executor | 路径执行器（direct 模式→直接发 /cmd_vel） |
| coverage_monitor | 覆盖率监控 |

**验证：**
```bash
ros2 service list | grep /coverage/
# 应看到 plan, start, pause, resume, stop 等服务
```

### 终端 4：控制

```bash
cd ~/mower_ws
source install/setup.bash

# 规划路径
ros2 service call /coverage/plan std_srvs/srv/Trigger
# 返回: 15917 个点, 6645.6m

# 开始执行
ros2 service call /coverage/start std_srvs/srv/Trigger
```

---

## 可视化说明

### RViz2 界面元素

| 元素 | 颜色 | 含义 |
|------|------|------|
| 🟩 绿色填充 + 边线 | 绿色 | 作业区域（46m×46m） |
| 🔷 蓝色路径线 + 箭头 | 蓝色 | 牛耕式全覆盖路径 |
| 🟢 绿色轨迹 | 绿色 | 已完成的覆盖路径段 |
| 🟣 紫色球 | 紫色 | 当前目标点 |
| ⚪ 白色点云 | 白色 | LiDAR 扫描的障碍物 |
| 🚗 小车模型 | — | 4WD 差速机器人 |

### 视角操作

| 操作 | 鼠标/键盘 |
|------|-----------|
| 旋转视角 | 左键拖拽 |
| 平移 | 中键拖拽 / Shift+左键 |
| 缩放 | 滚轮 / 右键拖拽 |
| 俯视图 | View 面板 → TopDownOrtho |
| 跟随小车 | Fixed Frame 设为 body |

---

## 常用命令

### 路径控制

```bash
# 规划路径
ros2 service call /coverage/plan std_srvs/srv/Trigger

# 开始执行
ros2 service call /coverage/start std_srvs/srv/Trigger

# 暂停（刀盘停止）
ros2 service call /coverage/pause std_srvs/srv/Trigger

# 恢复（断点续割）
ros2 service call /coverage/resume std_srvs/srv/Trigger

# 停止
ros2 service call /coverage/stop std_srvs/srv/Trigger

# 一键规划+执行
ros2 service call /coverage/plan_and_start std_srvs/srv/Trigger
```

### 监控

```bash
# 覆盖率统计
ros2 topic echo /coverage/statistics

# 查看话题列表
ros2 topic list

# 查看 TF 树
ros2 run tf2_tools view_frames

# 查看传感器频率
ros2 topic hz /odom
ros2 topic hz /velodyne_points
ros2 topic hz /imu/data
```

### 录制/回放

```bash
# 录制 60 秒（不含点云，节省空间）
ros2 bag record -o ~/bags/coverage_demo \
  /tf /tf_static /odom /joint_states /robot_description \
  /coverage/area_markers /coverage/path_markers \
  /coverage/execution_markers /coverage/coverage_map
# 按 Ctrl+C 停止

# 回放（2 倍速）
ros2 bag play ~/bags/coverage_demo --rate 2.0
```

---

## 故障排除

### ❌ `Frame [map] does not exist`

**原因：** 没有发布 map→odom 的 TF 变换。
**解决：**
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

### ❌ 点云不显示

**原因：** WSL2 上 Ignition 的 gpu_lidar 传感器偶尔会卡死。
**解决：** 重启仿真即可恢复。
```bash
# Ctrl+C 关掉 sim_launch.py 终端
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch outdoor_sim sim_launch.py
# 等待 20 秒后再重新规划+执行
```

### ❌ Robot Model 显示 Error

**原因：** 多次启动导致多个 `robot_state_publisher` 冲突。
**解决：** 
```bash
pkill robot_state_publisher
# 然后重新启动 sim_launch.py
```

### ❌ 编译后 launch 找不到节点

**原因：** `ament_python` 包没有自动创建 `lib/<pkg>/` 目录。
**解决：**
```bash
cd ~/mower_ws
for exe in area_definer boustrophedon_planner coverage_demo coverage_monitor path_executor; do
  ln -sf ../../bin/$exe install/mower_coverage/lib/mower_coverage/
done
```

### ❌ 仿真启动 1-2 分钟后崩溃

**原因：** WSL2 + 软件渲染的 Ignition Fortress 稳定性问题。
**解决：** 重启仿真，然后重新规划路径即可恢复：
```bash
# 终端 1：重启仿真
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch outdoor_sim sim_launch.py

# 终端 3：等待 20 秒后重新规划+执行
ros2 service call /coverage/plan std_srvs/srv/Trigger
ros2 service call /coverage/start std_srvs/srv/Trigger
```

> 💡 **实车提示**：在 RK3588 上使用 CycloneDDS + 硬件加速，不会出现上述崩溃问题。

### ❌ 小车越过目标点不停

已内置修复：当检测到机器人已经过目标点并开始远离时，自动跳到下一个点。

---

## 技术参数速查

| 参数 | 值 | 说明 |
|------|-----|------|
| 草坪大小 | 50m × 50m | 仿真世界 |
| 割幅 | 0.5 m | 割草宽度 |
| 重叠率 | 10% | 条带重叠 |
| 条带间距 | 0.45 m | 有效切割宽度 |
| 执行速度 | 1.0 m/s | 最大行进速度 |
| 路径点数 | ~15917 | 全覆盖路径 |
| 路径总长 | ~6646 m | 全路径长度 |
| 路径间距 | 0.3 m | 路径点采样 |
| 仿真物理 | 200 Hz | 物理引擎步长 |
| 渲染引擎 | ogre | 软件渲染（WSL2） |
