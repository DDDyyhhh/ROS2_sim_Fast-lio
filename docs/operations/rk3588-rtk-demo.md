# Mission: RK3588 RTK 演示

## Why

用户需要在 RK3588 上独立完成一次稳定、可解释、不会误触发电机的 RTK 演示，让领导看到 CORS 差分链路、UM982 固定解和 ROS 状态输出，而不是只看到一串坐标。

## Success looks like

- 能在 RK3588 上安全启动正式 RTK 只读 profile。
- 能解释 `/rtk/status` 中 `RTK_FIXED`、`ntrip=CONNECTED` 和 `global_position_trusted=true` 的含义。
- 能用 `/rtk/gps/fix` 频率、卫星数、HDOP 和 RTCM 字节数回答领导的现场问题。

## Constraints

- CORS 账号和密码只存在 RK3588 私有 `.env` 或环境变量中，不进入聊天、代码或日志。
- 演示期间电机电源关闭，不启动 CAN、规划器、执行器或 `/cmd_vel`。
- 讲义优先使用当前仓库和已完成现场验收证据。

## Out of scope

- 遥控采集作业区、禁区和跨区通道的实机运动。
- Mid-360 降级运动策略的现场验收。
- CAN 控制协议和电机动作。

## 现场资料

- [RK3588 RTK 部署说明](../../deploy/rk3588/README.md) — 镜像、Compose、环境变量和只读验证命令。
- [RTK 状态与实施交接](../../implementation-notes.md) — 当前验收证据、状态门槛和安全边界；不能替代现场输出。
- [UM982 用户手册](../hardware/um982/UM982_User_Manual.pdf) — 接收机串口、NMEA LOG 和配置命令的原始资料。
- [ROS 2 Humble command-line tools](https://docs.ros.org/en/humble/Concepts/Basic/About-Command-Line-Tools.html) — 官方命令行工具说明。

现场问题优先回到 RK3588 日志、`/rtk/status` 和实施交接证据；本演示不依赖社区操作。

## 已知缺口

目前没有领导专用的产品演示评分表；现场讲解以“链路连接、Fixed 解、数据新鲜度、无运动风险”四个可观察维度为准。
