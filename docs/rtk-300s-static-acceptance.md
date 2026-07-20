# RK3588 RTK 300 秒静态验收结果

## 一句话结论

在户外开阔环境、RTK 天线保持静止的条件下，RK3588 通过 CORS/NTRIP 将 RTCM
差分数据送入 UM982；最新一轮连续采集 300 条有效 GGA，300/300 条为
`RTK_FIXED`。

## 实测结果

| 指标 | 结果 |
| --- | --- |
| 测试类型 | 户外静态、天线保持不动 |
| 测试时长 | 300 秒 |
| RTCM 接收量 | 273681 字节 |
| 有效 GGA | 300/300 |
| 无效 GGA | 0 |
| RTK Fixed | 300/300（100%） |
| 首次进入 Fixed | 约 0.7 秒 |
| Fixed-only 东向标准差 | 0.008 m，约 8 mm |
| Fixed-only 北向标准差 | 0.010 m，约 10 mm |
| Fixed-only 径向标准差 | 0.008 m，约 8 mm |
| 相对首个 Fixed 的最大偏差 | 0.112 m |
| 最长连续 Fixed | 约 299 秒 |

## 现场展示方式

先展示这张历史验收结果，再在 RK3588 上展示当前状态：

```bash
docker exec mower-rkt bash -lc \
  'source /opt/ros/humble/setup.bash &&
   source /opt/mower_ws/install/setup.bash &&
   ros2 topic echo /rtk/status --once --field data'
```

当前输出重点应为 `state=RTK_FIXED`、`quality=4`、`ntrip=CONNECTED`、
`corrections_fresh=true` 和 `global_position_trusted=true`。

## 正确口径

这里的 8 mm、10 mm 是固定解样本的短时重复性/抖动统计，不是相对于测量基准点
的绝对定位误差，也不代表机器人运动中的最终导航精度。此次统计窗口从首次
`quality=4` 开始，因此相对首个 Fixed 样本的最大偏差 `0.112 m` 可能包含初始
收敛过程；不能据此宣称 300 秒内每个点都在 1 cm 范围内。运动验收还需要单独
评估 Mid-360、里程计、控制链和现场环境。

本次 profile 是只读 RTK 演示：不启动 CAN、规划器、执行器、电机或 `/cmd_vel`。
