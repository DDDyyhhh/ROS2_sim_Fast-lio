# 启动 Profiles

## Web 最小闭环

```bash
ros2 launch mower_coverage web_minimal.launch.py
```

启动斜坡仿真、Web HTTP 服务、rosbridge、多区域规划和 `direct` 执行器。默认 HTTP 端口为 8080，WebSocket 端口为 9090。

## 完整传感器闭环

```bash
ros2 launch mower_coverage sensor_full.launch.py
```

复用 `outdoor_sim/hill_full.launch.py`，启动仿真传感器、FAST-LIO、EKF、LaserScan、Web、规划和执行。

## 遥控采集仿真

```bash
ros2 launch mower_coverage remote_capture_sim.launch.py
```

启动 Gazebo、仿真 GNSS/RTK 天线状态、rosbridge、Web 和遥控采集向导；默认不启动规划器、执行器、CAN 或实机运动。确认作业区后，点击页面的“载入规划”，再点击底部“规划”。

定位为 `GREEN` 且位姿新鲜时，即使没有正在采集的对象，Web 摇杆也可以移动仿真机器人；只有点击“开始采集”后的 `capturing/draft` 状态会记录原始轨迹。地图上的黄色箭头和位姿文字显示 `/odom` 车头方向。

需要在仿真中继续测试规划和执行时，使用显式的规划 profile：

```bash
ros2 launch mower_coverage remote_capture_sim.launch.py \
  with_planning_execution:=true \
  with_lidar_scan:=true
```

此模式默认启动 `/scan` 安全链；执行命令仍经仿真命令仲裁器输出到 `/simulation/cmd_vel`，遥控输入是 `/simulation/teleop_cmd_vel`，自动路径输入是 `/simulation/plan_cmd_vel`，不发布全局 `/cmd_vel`。采集任务通过 `/web/mission` 进入新 planner，通道按已记录中心线和方向生成 transit 段；`/coverage/path_metadata` 标记通行段不计入覆盖率。`/scan` 缺失或不新鲜时仍会 fail-closed 停车。

## 旧入口

原有 `ros2 run mower_coverage ...` 和以下 launch 继续保留：

- `coverage_planning.launch.py`
- `hill_coverage.launch.py`
- `rosbridge_bridge.launch.py`
- `outdoor_sim` 中原有仿真与 Nav2 launch

## 运行时状态

新文件默认写入 `~/.local/state/mower_coverage/`。区域和 checkpoint 参数仍可显式覆盖；读取新路径失败时兼容旧的 `~/mowing_area.yaml`、`~/mowing_areas.yaml`、`~/hill_mowing_areas.yaml`、`~/coverage_checkpoint.json` 和 `~/hill_coverage_checkpoint.json`。
