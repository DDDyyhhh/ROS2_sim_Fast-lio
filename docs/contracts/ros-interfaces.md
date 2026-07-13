# ROS 接口契约

结构整理后继续保持以下公共接口：

- Topic：`/web/areas`
- Topic：`/coverage/multi_path`
- Topic：`/coverage/statistics`
- Service：`/multi_area/plan`
- Service：`/multi_area/plan_and_start`
- Service：`/multi_area/pause`
- Service：`/multi_area/resume`
- Service：`/multi_area/stop`
- Topic：`/cmd_vel`
- Topic：`/odom`
- Topic：`/scan`

以下目标规则仅作为后续行为改动的协议，本轮不实现：

1. 障碍物必须完全属于一个作业区域。
2. 作业区域之间禁止互相重叠。
3. 一批 Web payload 必须原子校验；非法时保留上一份有效任务。

`mid360-rtk-fixbase` worktree 中的安全状态机、坐标转换和传感器校验不合并到本轮。
