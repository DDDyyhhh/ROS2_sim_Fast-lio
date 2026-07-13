# 工作区架构

`/home/yh/mower_ws` 是唯一 Git 仓库和 ROS 2 工作区。

```text
mower_ws/
├── src/
│   ├── fast_lio/          # FAST-LIO，ROS 包名仍为 fast_lio
│   ├── outdoor_sim/       # 仿真、传感器桥接、EKF 与参考 Nav2 链路
│   └── mower_coverage/    # Web 多区域任务、规划与执行
├── docs/
│   ├── architecture/
│   ├── contracts/
│   ├── operations/
│   ├── daily/
│   └── archive/
└── build/ install/ log/   # 唯一一套 colcon 生成物
```

`mower_coverage` 的实现按工作流分层：

```text
mower_coverage/
├── mission/    # 区域任务定义与持久化
├── planning/   # 多区域覆盖规划
├── execution/  # direct/safe 路径执行
├── adapters/   # Web 等外部接口
└── legacy/     # 单区域、旧执行器和演示参考链路
```

本轮不引入新的 ROS 包，不修改 FAST-LIO 算法、规划算法或执行控制行为。
