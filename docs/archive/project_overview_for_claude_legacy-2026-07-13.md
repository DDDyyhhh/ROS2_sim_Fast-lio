# 自动割草机项目交接文档

> 历史交接文档（最后更新于 2026-07-13），保留用于追溯当时的软件闭环规划。文中“硬件未到货”等状态已经过时。
> 当前实施状态请以根目录 [`implementation-notes.md`](../../implementation-notes.md)、[`CLAUDE.md`](../../CLAUDE.md) 和任务计划为准。

---

## 1. 当前我们在做什么

### 当前阶段

项目处于 **硬件未到货前的软件闭环阶段**。

Mid360 LiDAR 和 RTK GPS 还没到，因此暂时不做真实点云、真实 RTK、实车标定和户外割草验证。当前重点是：

1. 把 **Web 前端 → rosbridge → 多区域/多障碍物路径规划 → 仿真执行** 做成稳定闭环。
2. 把系统整理成新会话、Codex、Claude Code 都能接手的结构化项目。
3. 在硬件到货前完成尽可能多的软件准备：仿真验收、坐标系契约、地图校验、任务状态机、安全逻辑、Mock 传感器和回归测试。

### 当前主线任务

| 优先级 | 任务 | 状态 |
|---|---|---|
| P0 | Web 端多区域/多障碍物绘制、发送、规划 | 已可用 |
| P0 | 黄线规划路径不得进入红色障碍物 | 已修复并测试 |
| P0 | `hill_full.launch.py` 一键启动 Web/rosbridge/规划系统 | 已修复并测试 |
| P1 | 建立项目日报系统 | 已建立 |
| P1 | 统一交接文档和 Claude 工作规则 | 进行中 |
| P1 | 仿真端到端任务闭环验收 | 下一步 |
| P2 | Mock RTK / Mock Mid360 | 待做 |
| P2 | 实车 RTK + Mid360 + EKF 融合 | 等硬件到货 |

---

## 2. 项目架构速览

### 仓库与工作区

```text
ROS 工作区: /home/yh/mower_ws
仓库路径:   /home/yh/mower_ws
主分支:     fix/web-launch-obstacle-planning
远程仓库:   git@github.com:DDDyyhhh/ROS2_sim_Fast-lio.git
```

不要把 `build/`、`install/`、`log/` 当源码改。源码在：

```text
/home/yh/mower_ws/src
```

### ROS 包

| 包 | 作用 |
|---|---|
| `outdoor_sim` | Ignition Fortress 仿真、URDF/world、桥接、EKF、hill 全量 launch |
| `fast_lio` | FAST-LIO LiDAR/IMU 里程计和建图，源码目录是 `src/fast_lio` |
| `mower_coverage` | Web 前端、rosbridge server、区域定义、牛耕式规划、路径执行 |

### 当前 Web 覆盖规划数据流

```text
浏览器 Leaflet 页面
  ↓ 画区域/障碍物
/web/areas
  ↓
multi_area_definer
  ↓ 写入
~/.local/state/mower_coverage/hill_mowing_areas.yaml
  ↓ /multi_area/plan
hill_boustrophedon
  ↓ 发布
/coverage/multi_path
  ↓ /multi_area/plan_and_start
multi_area_executor
  ↓
/cmd_vel
```

### 当前主要启动命令

```bash
cd /home/yh/mower_ws
source install/setup.bash
ros2 launch outdoor_sim hill_full.launch.py with_fastlio:=false with_ekf:=false with_lidar_scan:=false with_rosbridge:=true
```

浏览器打开：

```text
http://localhost:8080
```

rosbridge：

```text
ws://localhost:9090
```

---

## 3. 已完成什么

### 3.1 Web + rosbridge + 静态前端

已完成：

- `mower_coverage/web_frontend/index.html`
- `mower_coverage/web_frontend/app.js`
- `mower_coverage/mower_coverage/web_server.py`
- `mower_coverage/launch/rosbridge_bridge.launch.py`

已验证：

```text
8080: web_server 正常监听
9090: rosbridge_websocket 正常监听
curl http://localhost:8080/index.html -> HTTP 200
```

重要修复：

- `web_server.py` 改为可复用端口，避免 launch 快速重启后 `Address already in use`。
- 只有真正 bind 成功后才打印“正在监听”。
- 不再在线程里 `os.chdir()`，改用 `SimpleHTTPRequestHandler(directory=...)`。

回归测试：

```bash
python3 /home/yh/mower_ws/src/mower_coverage/tests/test_web_server.py
```

---

### 3.2 多区域、多障碍物路径规划

核心文件：

```text
src/mower_coverage/mower_coverage/hill_boustrophedon.py
src/mower_coverage/tests/test_hill_boustrophedon_obstacles.py
```

已修复：

1. 单区域单障碍物：黄色路径不再进入红色障碍物。
2. 单区域多障碍物：障碍物内部不再被黄色路径充满。
3. 多区域多障碍物：规划不再报 `plan_transit` 缺失。
4. 前端路径降采样不会删掉关键绕障点。
5. 路径点和线段都要避开膨胀后的障碍物。

关键策略：

- 障碍物先 `buffer(inner_inflate)` 膨胀。
- `work_area = area_polygon.difference(unary_union(obstacles))`。
- 后处理 `_fix_obstacle_crossings()` 删除障碍物内部点，并为穿越线段插入绕障点。
- `_downsample()` 保留速度变化点和会导致障碍物穿越的关键点。

回归测试：

```bash
python3 /home/yh/mower_ws/src/mower_coverage/tests/test_hill_boustrophedon_obstacles.py
```

已观察结果：

```text
web 端多区域多障碍物已成功规划
障碍物内无黄色路径穿过
```

---

### 3.3 日报系统

已建立：

```text
docs/daily/README.md
docs/daily/2026-07-09.md
```

规则：

当用户说“更新今天的日报”或类似指令时：

1. 更新当天 `docs/daily/YYYY-MM-DD.md`。
2. 更新 `docs/daily/README.md`。
3. 自动提交，但 **只提交 `docs/daily/` 下的日报文件**。
4. 不把代码改动混进日报提交。

已提交：

```text
191622e Add daily report index
3eb1136 Add daily report 2026-07-09
```

---

### 3.4 交接文档整理

当时的主交接文档：

```text
implementation-notes.md
CLAUDE.md
```

旧 Gemini 文档已归档：

```text
docs/archive/project_overview_for_gemini_legacy_2026-07-02.md
```

原因：旧 Gemini 文档是 2026-07-02 的 Phase 0/1 设计，包含较多旧流程，容易误导新会话。

已提交：

```text
dcb4aac Archive legacy Gemini handoff doc
```

---

## 4. 当前卡在哪

### 真正被硬件阻塞的内容

以下内容必须等 Mid360 / RTK 到货后才能最终验证：

| 阻塞项 | 原因 |
|---|---|
| Mid360 实机点云质量 | 需要真实 LiDAR 数据 |
| LiDAR/IMU 外参 | 需要真实安装位置和传感器数据 |
| RTK fix/float/single 状态 | 需要真实 RTK 设备和 NTRIP 环境 |
| RTK 天线杆臂 | 需要测量实体安装偏移 |
| EKF 实车融合参数 | 需要真实噪声、延迟、漂移 |
| 户外割草验证 | 需要实车、场地、安全测试 |

### 当前不该等硬件的内容

这些可以继续推进：

- 仿真端到端任务闭环。
- Web 地图绘制校验。
- 坐标系/单位/原点契约测试。
- 覆盖率、路径长度、转弯次数、预计耗时等任务指标。
- Mock RTK / Mock Mid360。
- 任务状态机。
- 软件安全 interlock。
- CI 或一键 smoke test。
- 操作员 Web 状态展示。

---

## 5. 下一步计划

推荐顺序：

### Step 1：做仿真端到端任务闭环

目标：形成一个可重复演示、可验收的完整流程。

验收标准：

```text
打开 http://localhost:8080
→ 画多个区域和障碍物
→ 发送到 ROS2
→ 点击规划
→ 生成无障碍物穿越路径
→ 启动执行
→ Web/ROS 显示执行状态
→ 输出任务摘要
```

建议新增指标：

- 路径总长度。
- 预计执行时间。
- 覆盖面积。
- 已覆盖比例。
- 跳过区域/异常区域。
- 障碍物安全距离。

---

### Step 2：加地图绘制校验

用户画错时要提前提示，不要等规划器报错。

需要校验：

- 区域多边形自交。
- 障碍物在区域外。
- 障碍物重叠。
- 区域太小。
- 通道太窄。
- 障碍物距离边界太近。
- 多区域命名/保存/清空逻辑。

---

### Step 3：整理坐标系契约

必须写清并测试：

```text
浏览器经纬度
GPS/RTK 经纬度
map 局部米制坐标
Ignition world 坐标
body/base 坐标
polygon/waypoint 坐标
```

尤其注意：

- `app.js` 和 `multi_area_definer.py` 里都有硬编码原点。
- 改原点必须两边同步，最好后续改成 ROS 参数或共享配置。

---

### Step 4：任务状态机和安全逻辑

建议状态：

```text
idle
map_loaded
validating
ready_to_plan
planning
planned
executing
paused
recovering
completed
failed
emergency_stop
localization_bad
sensor_stale
```

先在仿真和 Web 状态栏里跑通，不等硬件。

---

### Step 5：Mock RTK / Mock Mid360

硬件到货前准备接口：

- Mock `/gps/fix`。
- Mock RTK fix quality。
- Mock GPS 漂移、丢失、延迟。
- Mock `/velodyne_points` 或障碍物检测。
- Mock 传感器 stale/timeout。

目标：硬件到货后替换 publisher，而不是第一次联调整个系统。

---

## 6. 绝对不要再踩的坑

### 6.1 不要相信“进程在”就等于服务可用

之前 `web_frontend_server` 进程存在，但 8080 没监听。

必须验证：

```bash
ss -tlnp | grep -E ':(8080|9090)'
curl -I http://localhost:8080/
```

---

### 6.2 不要用 WSL 内部 IP 打开网页

Windows 浏览器访问：

```text
http://localhost:8080
```

不要用 WSL 的 `26.x.x.x` 或 `192.168.x.x`，可能超时。

---

### 6.3 不要让降采样破坏绕障路径

前端黄线是降采样后的 `/coverage/multi_path`。如果降采样删掉绕障关键点，前端会重新画出穿越障碍物的直线。

改 `_downsample()` 后必须跑：

```bash
python3 src/mower_coverage/tests/test_hill_boustrophedon_obstacles.py
```

---

### 6.4 不要只检查路径点，要检查线段

路径点不在障碍物内，不代表相邻点连线不穿越障碍物。

测试必须同时检查：

```text
Point inside obstacle: false
LineString crosses/within/contains obstacle: false
```

---

### 6.5 不要用全局障碍物合并框处理所有绕障

一个区域里多个障碍物时，用所有障碍物的全局 bounds 绕障，可能导致绕第一个障碍物时绕进第二个障碍物。

绕障时要基于实际阻挡当前线段的 obstacle 或 blocking union。

---

### 6.6 不要误删 `plan_transit()`

多区域规划需要区域间过渡：

```python
transit = self.plan_transit(wp[-1], next_poly)
```

如果 `plan_transit()` 缺失，单区域正常，多区域会崩。

---

### 6.7 不要把日报提交和代码提交混在一起

更新日报时只能提交：

```text
docs/daily/README.md
docs/daily/YYYY-MM-DD.md
```

不要顺手提交 `CLAUDE.md`、源码、worktree、计划文件或临时文件。

---

### 6.8 不要把旧 Gemini 文档当当前架构

旧文档在：

```text
docs/archive/project_overview_for_gemini_legacy_2026-07-02.md
```

它是历史参考，不是当前交接文档。

当前不再以本文为准；现在以这两个为准：

```text
implementation-notes.md
CLAUDE.md
```

---

### 6.9 不要盲信 README 里的旧参数

已有文档里存在历史信息，例如：

- `body` vs `base_link`
- 32线 vs 64线 LiDAR
- `mid360.yaml` vs `mid360_sim.yaml`
- 旧 coverage demo vs 当前 hill coverage stack

做修改前必须看当前源码、launch、config。

---

### 6.10 不要在 RK3588 上无限并行编译

必须限制：

```bash
export MAKEFLAGS="-j8"
```

避免 Orange Pi 5 Plus 编译时锁死。

---

## 7. 常用验证命令

### 路径规划回归

```bash
python3 /home/yh/mower_ws/src/mower_coverage/tests/test_hill_boustrophedon_obstacles.py
```

### Web server 回归

```bash
python3 /home/yh/mower_ws/src/mower_coverage/tests/test_web_server.py
```

### Python 语法检查

```bash
python3 -m py_compile \
  /home/yh/mower_ws/src/mower_coverage/mower_coverage/hill_boustrophedon.py \
  /home/yh/mower_ws/src/mower_coverage/mower_coverage/web_server.py
```

### Web/rosbridge 端口检查

```bash
ss -tlnp | grep -E ':(8080|9090)'
curl -I http://localhost:8080/
```

### 启动全系统最小模式

```bash
cd /home/yh/mower_ws
source install/setup.bash
ros2 launch outdoor_sim hill_full.launch.py with_fastlio:=false with_ekf:=false with_lidar_scan:=false with_rosbridge:=true
```

---

## 8. 近期重要提交

```text
81d04cb Fix web launch and obstacle path planning
191622e Add daily report index
3eb1136 Add daily report 2026-07-09
dcb4aac Archive legacy Gemini handoff doc
```

远程保存分支：

```text
origin/fix/web-launch-obstacle-planning
```

注意：本地可能比远程多日报和文档提交；推送前先检查：

```bash
git status --short --branch
git log --oneline -5
```
