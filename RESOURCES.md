# RK3588 RTK 演示 Resources

## Knowledge

- [RK3588 RTK 部署说明](deploy/rk3588/README.md) — 当前镜像、Compose、环境变量和只读验证命令；用于演示前准备与故障排查。
- [RTK 状态与实施交接](implementation-notes.md) — 当前真实验收证据、状态门槛和安全边界；用于解释演示结果，不能替代现场输出。
- [UM982 用户手册](docs/hardware/um982/UM982_User_Manual.pdf) — 接收机串口、NMEA LOG 和配置命令的原始资料；用于回答串口与设备问题。
- [ROS 2 Humble command-line tools](https://docs.ros.org/en/humble/Concepts/Basic/About-Command-Line-Tools.html) — ROS 2 官方命令行工具说明；用于解释 `ros2 topic echo/hz`。

## Wisdom (Communities)

- 当前 mission 不依赖社区操作；现场问题优先回到 RK3588 日志、`/rtk/status` 和实施交接证据。

## Gaps

- 目前没有领导专用的产品演示评分表；本 lesson 使用“链路连接、Fixed 解、数据新鲜度、无运动风险”四个可观察维度。
