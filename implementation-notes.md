# Deviations

- 根目录已经存在无效的 `.git` 目录，而计划假设 Git 根仅位于 `src/ROS2_sim_Fast-lio`。迁移前先盘点其内容，避免覆盖未知用户状态。
- Codex 沙箱把根 `.git` 挂为只读，用户在普通终端完成 Git 元数据移动后继续；源码迁移未在中间态执行，仓库历史保持完整。
- 为确保旧 `ros2 run` 真正可用，新增标准 `setup.cfg` 将脚本安装到 `lib/mower_coverage`；原计划只提到 `setup.py` 映射，但仅映射不足以被 ROS 2 枚举。
- 当前环境没有 `pytest` 命令，改为直接运行三个现有独立测试脚本；没有联网安装依赖。

# Discovered edge cases

- 当前仓库登记了 5 个 worktree：主 worktree、外部 mid360 worktree，以及 3 个嵌套在主仓库 `.claude/worktrees` 下的 worktree。
- 根目录同时存在标准与 mid360 专用的多套 build/install/log，且嵌套仓库自身也有 build/log。
- 手工发送非 Web 契约格式（`points` 为数组而非 `{x,y,z}` 对象）会让 `multi_area_definer` 回调异常退出；原子校验属于计划明确排除的后续行为改动，本轮仅记录。
- `sensor_full` 的所有进程可启动，但现有链路报告 `/scan` QoS 不兼容、缺少 `base_link -> gps_link` TF，以及 FAST-LIO 点云缺少 `time` 字段；这些属于计划明确不在本轮修改的传感器/行为问题。

# Questions for review

- 是否在后续传感器专项中修复 `/scan` QoS、`base_link -> gps_link` TF 和点云 `time` 字段？

# Verification evidence

- `colcon list` 只发现 `fast_lio`、`mower_coverage`、`outdoor_sim`，全量 `colcon build --symlink-install` 成功。
- 9 个旧 console scripts 可由 `ros2 pkg executables mower_coverage` 枚举；3 个旧 launch 与 2 个新 profile 均可解析。
- 三个现有测试脚本通过；Web 测试因沙箱 socket 限制在获准的本地环境运行。
- `web_minimal` 验收：HTTP 200、rosbridge 9090、真实 `/web/areas` payload、规划 1188 点、执行 197 点、`/coverage/statistics` 输出执行状态。
- 工作区内仅有根 `.git` 和根 build/install/log；4 个附加 worktree 与历史生成物均在工作区外保留。

# Summary

- Deviations count: 3.
- Most likely revisit: 完整传感器 profile 暴露的 QoS、TF 与点云字段问题。
- Edge cases found: 4（只读 `.git` 挂载、ROS Python 脚本安装位置、非法 Web points 格式、现有传感器链警告）。
- Verification status: 结构、构建、兼容入口和 Web 最小闭环通过。
- Next session should read first: `docs/architecture/workspace.md` 与本文件的 Questions for review。

## Active Handoff（当前交接进度）

- 当前进度：工作区根迁移、三个 ROS 包整理、`mower_coverage` 工作流分层、旧入口兼容、新启动 profiles、运行时状态路径和分层文档均已完成。
- 当前提交：`06dfec8 Reorganize mower workspace and ROS profiles`；当前分支为 `fix/web-launch-obstacle-planning`。
- 验证状态：全量构建、现有回归测试、旧入口与 launch 解析、`web_minimal` Web 多区域规划执行闭环均已通过；`sensor_full` 的所有目标进程可以启动。
- 保留状态：附加 worktree 位于 `/home/yh/mower_ws-worktrees/`，历史生成物位于 `/home/yh/mower_ws-archive/2026-07-13/`。
- 工具清理：确认项目内 `.agents/skills` 与全局 skills 完全一致后，已删除 `.agents/`、`.superpowers/`、`docs/superpowers/plans/` 和 `skills-lock.json`。
- 已知问题：`sensor_full` 存在 `/scan` QoS 不兼容、缺少 `base_link -> gps_link` TF、FAST-LIO 点云缺少 `time` 字段；非法 Web points 格式仍可能导致定义节点退出。
- 下一步行动：单独规划并实施传感器链修复，优先处理 `/scan` QoS、GPS TF 和点云时间字段；之后再规划 Web payload 原子校验与安全状态机，避免混入本次结构迁移提交。
