# Mission: RK3588 RTK 演示

## Why

用户需要在 RK3588 上独立完成一次稳定、可解释、不会误触发电机的 RTK 演示，让领导看到 CORS 差分链路、UM982 固定解和 ROS 状态输出，而不是只看到一串坐标。

## Success looks like

- 能在 RK3588 上安全启动正式 RTK 只读 profile。
- 能解释 `/rtk/status` 中 `RTK_FIXED`、`ntrip=CONNECTED` 和 `global_position_trusted=true` 的含义。
- 能用 `/gps/fix` 频率、卫星数、HDOP 和 RTCM 字节数回答领导的现场问题。

## Constraints

- CORS 账号和密码只存在 RK3588 私有 `.env` 或环境变量中，不进入聊天、代码或日志。
- 演示期间电机电源关闭，不启动 CAN、规划器、执行器或 `/cmd_vel`。
- 讲义优先使用当前仓库和已完成现场验收证据。

## Out of scope

- 遥控采集作业区、禁区和跨区通道的实机运动。
- Mid-360 降级运动策略的现场验收。
- CAN 控制协议和电机动作。
