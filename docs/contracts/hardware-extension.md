# 硬件扩展位置

本轮不实现真实硬件驱动。硬件 profile 接入时必须提供或适配以下既有接口：

- 定位：`/odom` 与完整 TF 树
- 2D 避障：`/scan`
- 执行命令：订阅 `/cmd_vel`
- 完整传感器验证：`/velodyne_points`、`/imu/data`、`/gps/fix`

真实驱动应放在独立硬件包或 adapter 中，不进入规划器和执行器核心实现。
