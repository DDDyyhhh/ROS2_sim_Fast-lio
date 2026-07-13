# Deviations

- 根目录已经存在无效的 `.git` 目录，而计划假设 Git 根仅位于 `src/ROS2_sim_Fast-lio`。迁移前先盘点其内容，避免覆盖未知用户状态。
- Codex 沙箱把根 `.git` 挂为只读，用户在普通终端完成 Git 元数据移动后继续；源码迁移未在中间态执行，仓库历史保持完整。
- 为确保旧 `ros2 run` 真正可用，新增标准 `setup.cfg` 将脚本安装到 `lib/mower_coverage`；原计划只提到 `setup.py` 映射，但仅映射不足以被 ROS 2 枚举。
- 当前环境没有 `pytest` 命令，改为直接运行三个现有独立测试脚本；没有联网安装依赖。
- 交接只列出 `/scan` QoS、GPS TF 和点云 `time` 三项；运行 `sensor_full` 后发现 `hill_nav2_local` 把 `cloud_in` 重映射写成了无效的 `/velodyne_points` 自映射，且 EKF 输出仍使用默认 `base_link`。两项均直接阻断同一传感器融合链，因此纳入本专项的最小修复范围。
- 修复 TF 后进一步检查发现 `navsat_transform` 默认发布 `/odometry/gps`，而现有 EKF 契约订阅 `/odom/gps`；补充显式输出 remap，避免“TF 已修好但 GPS 未进入融合”的假闭环。
- 为处理仿真高负载下的时间点 TF 查询，新增 navsat `transform_timeout: 0.2`；这是可逆的配置级修复，未改变 TF 拓扑或融合权重。

# Discovered edge cases

- 当前仓库登记了 5 个 worktree：主 worktree、外部 mid360 worktree，以及 3 个嵌套在主仓库 `.claude/worktrees` 下的 worktree。
- 根目录同时存在标准与 mid360 专用的多套 build/install/log，且嵌套仓库自身也有 build/log。
- 手工发送非 Web 契约格式（`points` 为数组而非 `{x,y,z}` 对象）会让 `multi_area_definer` 回调异常退出；原子校验属于计划明确排除的后续行为改动，本轮仅记录。
- `sensor_full` 的所有进程可启动，但现有链路报告 `/scan` QoS 不兼容、缺少 `base_link -> gps_link` TF，以及 FAST-LIO 点云缺少 `time` 字段；这些属于计划明确不在本轮修改的传感器/行为问题。
- 真实仿真点云字段为 `x,y,z,intensity,ring`，组织尺寸为 `64×1200`，没有 `time`；`pointcloud_to_laserscan` 当前没有订阅 `/velodyne_points`，导致 `/scan` 只有发布端没有消息。
- `/scan` 发布端为 BEST_EFFORT、执行器订阅端为 RELIABLE；`/odometry/filtered` 的 child frame 为 `base_link`，而 URDF 与 FAST-LIO 使用 `body`。
- navsat 节点实际暴露 `/odometry/gps`，原 launch 没有把它接到配置声明的 `/odom/gps`，导致 `/odom/gps` 没有发布者；修复后需验证 GPS 输出与 EKF 输入相连。
- ROS 2 Humble 的 `navsat_transform` 参数列表不包含 `base_link_frame`；GPS 外参 frame 由其订阅的 filtered odometry `child_frame_id` 推导，故只在 EKF 配置中设置 `base_link_frame: body`，不保留无效 navsat 参数。
- 清洁运行中前约 30 秒 GPS 输出正常，随后出现 `Could not obtain odom -> body transform`；这是 `navsat_transform` 默认 `transform_timeout=0` 在仿真高负载下无法等待对应时间点 TF 的新暴露问题，先采用可逆的 0.2 秒等待窗口验证。
- 最终清洁运行约 60 秒仍有 4 条 EKF “Failed to meet update rate” 提示；未见传感器契约错误，属于后续性能专项，不在本次修复中调参。

# Questions for review

- 已处理：`/scan` QoS、`body -> gps_link` frame 契约、PointCloud2 缺失 `time`、点云输入 remap、GPS 输出 remap，以及 navsat TF 等待窗口。
- 后续性能专项是否需要处理仿真高负载下 EKF 30 Hz 更新率提示，以及长时间运行中偶发的 FAST-LIO `No Effective Points`？

# Verification evidence

- `colcon list` 只发现 `fast_lio`、`mower_coverage`、`outdoor_sim`，全量 `colcon build --symlink-install` 成功。
- 9 个旧 console scripts 可由 `ros2 pkg executables mower_coverage` 枚举；3 个旧 launch 与 2 个新 profile 均可解析。
- 三个现有测试脚本通过；Web 测试因沙箱 socket 限制在获准的本地环境运行。
- `web_minimal` 验收：HTTP 200、rosbridge 9090、真实 `/web/areas` payload、规划 1188 点、执行 197 点、`/coverage/statistics` 输出执行状态。
- 工作区内仅有根 `.git` 和根 build/install/log；4 个附加 worktree 与历史生成物均在工作区外保留。
- 本次传感器修复前的 `sensor_full` 运行基线已复现三项交接错误及点云 remap、EKF frame 两项关联错误。
- 新增传感器契约用例先在基线失败（首次 4 项，发现 GPS 输出断链后扩展为 5 项），最终 5 项全部通过；Python 语法与 `git diff --check` 通过。
- 最终 `colcon build --symlink-install --packages-select fast_lio mower_coverage outdoor_sim` 成功；既有规划器、障碍物和 Web socket 回归也通过（Web socket 在获准本地环境运行）。
- 最终清洁 `sensor_full` 运行：`/scan` 约 8–9 Hz、`/odom/gps` 约 4 Hz；`/scan` 端点均 BEST_EFFORT、`/odom/gps` 有 navsat 发布者且被 EKF 订阅；日志中的四类传感器/TF 错误均为 0。

# Summary

- Deviations count: 7（含本轮发现并保守修复的 cloud_in、GPS 输出和 TF 等待配置偏差）。
- Most likely revisit: 仿真高负载下 EKF 30 Hz 更新率与 FAST-LIO 长时间有效点质量。
- Edge cases found: 10（含只读挂载、脚本安装、非法 Web payload、传感器字段/QoS/remap、frame、GPS 输出与 TF 时间窗口）。
- Verification status: 传感器契约用例、构建、既有回归、端点频率/QoS/TF 运行验收均通过；仅保留性能提示。
- Next session should read first: 本文件的 Questions for review 与 Active Handoff；Web payload 原子校验仍未开始。

## Active Handoff（当前交接进度）

- 当前进度：工作区根迁移、三个 ROS 包整理、`mower_coverage` 工作流分层、旧入口兼容、新启动 profiles、运行时状态路径、分层文档和 `sensor_full` 传感器链专项均已完成。
- 当前提交：`06dfec8 Reorganize mower workspace and ROS profiles`；当前分支为 `fix/web-launch-obstacle-planning`。
- 验证状态：全量构建、5 项传感器契约用例、现有规划/障碍物/Web 回归、旧入口与 launch 解析、`web_minimal` Web 多区域规划执行闭环均已通过；清洁 `sensor_full` 运行中 `/scan`、`/odom/gps` 有数据，四类传感器/TF 错误为 0。
- 保留状态：附加 worktree 位于 `/home/yh/mower_ws-worktrees/`，历史生成物位于 `/home/yh/mower_ws-archive/2026-07-13/`。
- 工具清理：确认项目内 `.agents/skills` 与全局 skills 完全一致后，已删除 `.agents/`、`.superpowers/`、`docs/superpowers/plans/` 和 `skills-lock.json`。
- 已知问题：清洁运行仍有少量 EKF “Failed to meet update rate” 性能提示；长时间高负载运行曾出现 FAST-LIO `No Effective Points`，非法 Web points 格式仍可能导致定义节点退出。
- 本次实现：修正 `cloud_in → /velodyne_points`；safe 执行器使用 sensor-data BEST_EFFORT QoS；EKF 使用 `body`，navsat 输出 remap 到 `/odom/gps` 并等待 TF 0.2 秒；FAST-LIO 对无 `time` 但有 `ring` 的 PointCloud2 使用无时间字段解析并复用 yaw 合成 offset time。
- 下一步行动：本专项无需继续修改；如继续开发，先决定是否单独开启传感器性能专项，再规划 Web payload 原子校验与安全状态机。本次改动已提交为 `b69dbea` 并推送到当前远程分支。
