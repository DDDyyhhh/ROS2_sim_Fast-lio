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

## 旧入口

原有 `ros2 run mower_coverage ...` 和以下 launch 继续保留：

- `coverage_planning.launch.py`
- `hill_coverage.launch.py`
- `rosbridge_bridge.launch.py`
- `outdoor_sim` 中原有仿真与 Nav2 launch

## 运行时状态

新文件默认写入 `~/.local/state/mower_coverage/`。区域和 checkpoint 参数仍可显式覆盖；读取新路径失败时兼容旧的 `~/mowing_area.yaml`、`~/mowing_areas.yaml`、`~/hill_mowing_areas.yaml`、`~/coverage_checkpoint.json` 和 `~/hill_coverage_checkpoint.json`。
