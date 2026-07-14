# Deviations

- 根目录已经存在无效的 `.git` 目录，而计划假设 Git 根仅位于 `src/ROS2_sim_Fast-lio`。迁移前先盘点其内容，避免覆盖未知用户状态。
- Codex 沙箱把根 `.git` 挂为只读，用户在普通终端完成 Git 元数据移动后继续；源码迁移未在中间态执行，仓库历史保持完整。
- 为确保旧 `ros2 run` 真正可用，新增标准 `setup.cfg` 将脚本安装到 `lib/mower_coverage`；原计划只提到 `setup.py` 映射，但仅映射不足以被 ROS 2 枚举。
- 当前环境没有 `pytest` 命令，改为直接运行三个现有独立测试脚本；没有联网安装依赖。
- 交接只列出 `/scan` QoS、GPS TF 和点云 `time` 三项；运行 `sensor_full` 后发现 `hill_nav2_local` 把 `cloud_in` 重映射写成了无效的 `/velodyne_points` 自映射，且 EKF 输出仍使用默认 `base_link`。两项均直接阻断同一传感器融合链，因此纳入本专项的最小修复范围。
- 修复 TF 后进一步检查发现 `navsat_transform` 默认发布 `/odometry/gps`，而现有 EKF 契约订阅 `/odom/gps`；补充显式输出 remap，避免“TF 已修好但 GPS 未进入融合”的假闭环。
- 为处理仿真高负载下的时间点 TF 查询，新增 navsat `transform_timeout: 0.2`；这是可逆的配置级修复，未改变 TF 拓扑或融合权重。
- 交接建议只重构 `plan_from_areas()` 的 seam，但红灯继续暴露出 seam 起点贴近障碍物下边界时现有绕障候选全部失败；因此补充“先向外下移再横向”的候选，并让安全余量允许明确远离障碍物的首段，范围仍限于共享绕障判定逻辑。
- 区域间连接改为“前一区域末点 → 后一区域首个覆盖点”，不再插入可能落入膨胀障碍物的最近边界点；保留 `plan_transit()` 作为已有接口，未删除或扩展其签名。
- 代码审查发现 seam 绕障失败时若继续追加后续覆盖点，会重新生成未验证的跨障线段；增加 fail-closed 检查，无法安全连接时让本次规划失败而不发布不完整路径。

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
- 用户在 `sensor_full` 的 RViz2 中复现小车闪烁；截图显示 `body/gps_link/imu_link/lidar_link` 可变换，但四个轮子均为 `No transform`。静态检查发现该 profile 同时存在 Ignition DiffDrive `/tf`、`odom_to_tf` 中继和 EKF `odom → body` 发布路径，且 `/joint_states` 轮关节桥接尚未有运行态确认。
- 运行态确认 `/tf` 的 4 个发布者为 `robot_state_publisher`、`laser_mapping`、`ekf_localization`、`odom_to_tf`；`/odom` 正常约为 `[-1.61, -3.43, 0]`，但 `/odometry/filtered` 已发散到 `-1.1e10` 位置和 `-1.6e8` 速度，故闪烁直接来自错误 EKF TF 与正常 `odom_to_tf` TF 的竞争。`/joint_states` 确实包含四个轮关节；同时运行态 `/odom/gps` 无发布者且节点列表缺少 `navsat_transform`，GPS 融合链仍不稳定。
- 用户新增产品优先级：Web 路径在障碍物 2 附近显示稀疏，且出现穿越障碍物。规划器默认割幅间距为 `0.45m`（`0.5m` 割幅、10% 重叠），Web 还将路径按每 10 个点降采样；“稀疏”需区分覆盖间距、可视化降采样和真实路径点，不能直接改参数。障碍物穿越属于执行安全阻断，优先级高于纯 RViz 显示问题；初步代码线索是区域间 `plan_transit()` 生成直线后没有对全局障碍物再次做碰撞检查。
- 本轮最小复现确认：第二个区域边界附近的障碍物膨胀后可延伸到区域外，`plan_from_areas()` 仍把第一个区域终点直连到第二个区域边界，且全局 `all_obstacles` 只在 transit 追加后才赋给 `last_obstacles`；新增的跨区域 transit 回归目前按预期失败。
- 代码审查进一步发现：全局绕障只检查候选点会让候选竖直/水平线段穿过另一个分离障碍物；已用两个障碍物的完整 `plan_from_areas()` 场景复现，补充第二条红灯回归后改为逐段验证候选绕障链。
- 本轮新复现：Web 截图对应的区域 2 单独规划原始路径从 415 点恢复到 1974 点、最高到 y≈25.9；原因是 `difference/intersection` 浮点误差让扫描线端点约 2.7e-9m 落入膨胀障碍物，后处理从同一旧端点反复失败。当前未提交的 `combined.buffer(1e-4)` 探索性改动可修复这一层。
- 本轮进一步确认：两个带障碍物区域一起调用 `plan_from_areas()` 时，局部路径分别为约 960/1974 点，但最终全局 `_fix_obstacle_crossings()` 从 2936 点删到 960 点，区域 2 变为 0 点；全局流式后处理在 transit 失败后保留区域 1 末点，把后续区域 2 点全部当成同一条待绕障线段处理并跳过。
- 本轮 seam 修复后发现：区域 1 末点仅在自身膨胀障碍物外约 `0.0001m`，旧安全余量会把向下离开的垂直线段误判为危险；只放宽“远离障碍物”的首段即可恢复安全连接，不需要删除区域覆盖点。
- 本轮审查新增边界：`_fix_obstacle_crossings()` 在极端失败时会返回起点；seam 调用必须验证终点已到达首个覆盖点，否则不能继续拼接后续点。

# Questions for review

- 已处理：`/scan` QoS、`body -> gps_link` frame 契约、PointCloud2 缺失 `time`、点云输入 remap、GPS 输出 remap，以及 navsat TF 等待窗口。
- 后续性能专项是否需要处理仿真高负载下 EKF 30 Hz 更新率提示，以及长时间运行中偶发的 FAST-LIO `No Effective Points`？
- `sensor_full` 的 RViz2 车体闪烁是否需要单独专项处理：优先确认唯一 `odom → body` TF 发布者、验证 `/joint_states` 的四个轮关节消息，再补最小 RViz/TF 回归验收。
- 路径专项的验收必须先证明完整全局路径（包括区域间 transit 和前端降采样后的 `/coverage/multi_path`）不与任一膨胀障碍物相交，再调整路径密度参数。
- 本轮路径专项已建立最小红灯：`test_multi_area_transit_avoids_next_area_obstacle`；修复前失败点为第二个区域左边界的 transit 终点进入膨胀障碍物，修复后需同时复验原始路径与降采样路径。
- 本轮审查新增红灯：`test_global_detour_avoids_all_separated_obstacles`；修复前命中 `(5.3, 1.9) → (2.7, -0.2)`，说明候选点安全不等于候选线段安全。
- 本轮新增红灯：`test_angled_obstacle_does_not_delete_area_two_coverage` 与 `test_two_areas_keep_second_area_coverage_with_obstacles`，均已在 seam 修复后通过，并复验降采样路径安全。
- 本轮建议已落实：保留每个区域已通过局部障碍检查的 `wp`，只对“前一区域末点 → 后一区域首个覆盖点”的 transit 连接做全局碰撞修复；不再对拼接后的所有区域路径统一流式调用 `_fix_obstacle_crossings()`，全局障碍物在连接前已收集。

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
- 本轮路径回归：跨区域 transit 与分离障碍物候选线段用例均先失败后通过；6 项障碍物脚本回归、12 项 `mower_coverage` 包测试、Python 语法检查和 `git diff --check` 均通过。Web socket 测试首次受沙箱限制，获准本地环境重跑通过。
- 本轮构建：`colcon build --symlink-install --packages-select fast_lio mower_coverage outdoor_sim` 成功。
- 本轮新验证：单区域斜障碍回归先失败（415 点）后通过（epsilon 后 1974 点）；双区域真实回归先出现区域 2 为 0 点，完成 seam 修复后覆盖与全局安全断言均通过。
- 本轮新验证：8 项障碍物路径回归、5 项旧规划器回归、Python 语法检查、`git diff --check`、三包构建和 Web 回归均通过；`colcon test` 最终为 14 tests / 0 errors / 0 failures。
- 本轮审查：按用户 AGENTS 的 Standards 和本交接 spec 手工完成双轴 review；未发现阻断性 standards/spec finding。当前工具未提供该 skill 要求的并行 sub-agent，因此未伪造子代理报告。

# Summary

- Deviations count: 10（本轮新增 seam 绕行候选、方向感知安全余量判定和 fail-closed 连接检查，均为局部安全修复）。
- Most likely revisit: 仿真高负载下 EKF 30 Hz 更新率与 FAST-LIO 长时间有效点质量。
- Edge cases found: 17（新增扫描线端点浮点内缩导致后处理卡死、全局 transit 失败吞掉后续区域路径、seam 起点贴边导致安全余量拒绝向外连接，以及 seam 失败后继续拼接的未验证线段）。
- Verification status: 本专项当前验证全绿：8 项障碍物路径回归、5 项旧规划器回归、14 项 `colcon test`、三包构建、语法检查、diff 检查和 Web socket 回归均通过；仅保留交接中已知的性能/传感器后续问题。
- Next session should read first: 本文件的 Questions for review 与 Active Handoff；路径 seam 专项已完成，后续若继续应先决定是否开启传感器性能或 Web payload 安全专项。

## Active Handoff（当前交接进度）

- 当前进度：已完成“双区域各有障碍物时区域 2 覆盖丢失”的 seam 修复；区域内路径保留，区域间连接单独执行全局绕障，针对性回归已通过，等待完整验证。
- 当前提交：`HEAD`（`Preserve multi-area coverage during obstacle transit`）；当前分支为 `fix/web-launch-obstacle-planning`。
- 验证状态：上一轮全量构建、5 项传感器契约用例、6 项障碍物路径回归、12 项包测试和 Web 回归均通过；本轮 8 项障碍物路径回归、14 项 `colcon test`、三包构建和 Web 回归均通过。
- 保留状态：附加 worktree 位于 `/home/yh/mower_ws-worktrees/`，历史生成物位于 `/home/yh/mower_ws-archive/2026-07-13/`。
- 工具清理：确认项目内 `.agents/skills` 与全局 skills 完全一致后，已删除 `.agents/`、`.superpowers/`、`docs/superpowers/plans/` 和 `skills-lock.json`。
- 已知问题：清洁运行仍有少量 EKF “Failed to meet update rate” 性能提示；长时间高负载运行曾出现 FAST-LIO `No Effective Points`；`sensor_full` 的 RViz2 车体闪烁与四轮 `No transform` 尚未修复；非法 Web points 格式仍可能导致定义节点退出。
- 本次实现：完成两阶段区域规划；先收集全部膨胀障碍物，再只对相邻区域首尾连接调用绕障；增加下方绕行候选、远离障碍物的贴边首段判定和 seam fail-closed 检查。测试文件未再扩展。
- 下一步行动：本专项验证与 review 已完成；提交当前规划器、回归测试和实施笔记改动。`sensor_full` 专项顺延。
- 当前会话进度：代码审查发现的 seam 失败分支已修正，完整验证全绿；当前提交为 `HEAD`，规划器、回归测试和实施笔记已提交。
