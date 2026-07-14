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
- 本轮运行态诊断需要 DDS/Gazebo 的本地网络接口与可写日志目录；沙箱内启动会失败，因此仅将 ROS 日志重定向到 `/tmp`，真实 ROS 诊断命令经获准的本地环境执行，未把环境错误当作代码结论。
- 为验证重复 TF，曾在一个临时运行实例中停止 `odom_to_tf`；该 launch 随后整体退出，第一次 datum 服务探针没有得到有效结果。重新启动干净实例后才完成 datum 服务验证，源码和工作区状态未被该探针改变。
- 交接原计划只要求确认唯一 `odom → body` owner；运行态同时证实仿真 GPS 未配置 datum 会把 `/odom/gps` 推到百万米级，因此将“为仿真固定 GPS 配置正确 datum”纳入同一 TF 稳定性最小修复，真实硬件 GPS 仍不改。
- Web 输入错误的“明确错误”沿用现有单向 `/web/areas` topic 能力，以 `multi_area_definer` 的 error log 返回；未新增响应 topic 或修改前端协议，避免把一次性安全修复扩展成新的通信契约。
- 完整 profile 实际 EKF 与 FAST-LIO 输出约 20 Hz；将仿真 EKF 频率从 30 Hz 调整到 20 Hz，只匹配已测得的输入负载，未改变融合变量或硬件配置。
- 本轮诊断时当前 ROS/Gazebo 进程不在本会话可见的进程表中，因此没有擅自启动第二个仿真实例；结论基于同一工作区保存的完整运行日志和可重复的内存控制探针，下一窗口必须补一次实时 topic 快照。

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
- 本轮新增 Web/性能边界：一次 payload 中任一点为数组、坐标非有限数字、名称非字符串或多边形无效都必须在状态替换前拒绝；完整 Web+Scan 长跑会让 FAST-LIO 进入无有效点循环，不能只用 EKF 频率判断传感器健康。
- 包级测试在 Codex 沙箱内运行时，Web 端口复用用例会因创建临时 TCP socket 被拒绝而报 `PermissionError`；同一用例及完整 `mower_coverage` 测试在获准本地环境通过，不能把该沙箱限制当作代码回归。
- `sensor_full` 历史长跑 `/tmp/mower_sensor_full_final_20hz.log` 统计到约 43,048 次 `lidar loop back, clear buffer`、8,475 次 `No point, skip this scan` 和 230 次 `No Effective Points`；这是点云时间顺序/处理负载链的强证据，不应继续只调 EKF 频率。
- `clock_filter.py` 只保证 `/clock_raw` → `/clock` 单调，不会改写 `/velodyne_points.header.stamp`；FAST-LIO 的 PointCloud2 回调遇到回退时间会清空 buffer 后仍接收旧帧，形成“回退→丢帧→无有效点”的候选闭环。
- `MultiAreaExecutor` 的 safe 逻辑在前方 ±30° 命中 `<0.8m` 有效点时不进入目标跟踪；内存探针持续注入正前方 0.5m 点，40 次循环前进速度为 0，末次命令为 `linear=-0.3, angular=0.6`。
- 规划器路径和执行器/前端位置的契约仍分裂：路径 `frame_id=map`，`map→odom` 当前为单位变换，但执行器和网页都订阅原始 `/odom`，没有使用 `/odometry/filtered`；需要用同一时刻坐标和速度实测后再决定是否改为融合定位。

# Questions for review

- 已处理：`/scan` QoS、`body -> gps_link` frame 契约、PointCloud2 缺失 `time`、点云输入 remap、GPS 输出 remap，以及 navsat TF 等待窗口。
- 后续性能专项仍需处理长时间高负载下 FAST-LIO `No Effective Points` / `No point, skip` 与由此造成的 EKF 阻塞；20 Hz 只解决 EKF 与 FAST-LIO 输入速率不匹配，完整长跑仍出现少量 update-rate failure，不能宣称性能专项完成。
- 已处理：`sensor_full` 的 RViz2 车体闪烁。修复后 `/tf` 仅保留 EKF、FAST-LIO、robot_state_publisher 三个 publisher，`/tf_static` 仅一个 `map → odom`，四轮 frame pair 持续发布。
- 若产品需要浏览器直接显示 Web payload 错误，需要另行确认响应 topic/服务契约；当前实现按现有单向 `/web/areas` 协议记录明确 error log，不向浏览器发送新消息。
- 新问题的下一步应先固定点云时间单调性和执行器实际停车原因，再处理 EKF 参数；不要在没有 `/scan`、`/velodyne_points` 时间戳和 `/cmd_vel` 实测数据时直接降低滤波频率或放宽安全阈值。
- 产品/架构待确认：覆盖执行与网页机器人位置应继续使用仿真原始 `/odom`，还是统一改用 EKF `/odometry/filtered`；前者便于仿真验收，后者才符合当前融合定位链，不能默默混用。
- 下一窗口需要 A/B 验证：关闭 FAST-LIO/EKF、关闭 `/scan` 或使用 direct 执行模式分别测试“控制器能否驶向首个目标点”；每次只改变一个开关并记录 `/cmd_vel`、`/odom`、`/scan` 和执行状态。
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
- 本轮 RViz/TF 诊断：干净 `sensor_full` 运行中 `/tf` 有 `robot_state_publisher`、`laser_mapping`、`ekf_localization`、`odom_to_tf` 四个 publisher；frame-pair probe 观察到 `odom → body` 持续发布，停止 `odom_to_tf` 后 publisher 降为 3 个且该 pair 仍存在，确认 EKF 是保留 owner。`/joint_states` 同时包含四个轮关节。
- 本轮 GPS 诊断：默认 navsat 日志为 Datum `(0,0,0)`，`/odom/gps` 约为 `(31368,2495923)`；通过 `/datum` 注入仿真 GPS 的 `22.5431,114.0579` 后，`/odom/gps` 回到约 `(0.04,0.18)`，直接验证原点错配。由于 EKF 已在坏状态运行，旧实例的 z 状态仍不可用于验收，需在代码修复后清洁重启验证。
- 本轮修复验证：先新增 3 项 sensor-chain 红灯，再修改 `hill_sim_launch` 的中继开关、`hill_full` 的默认 owner、`hill_ekf` 的 datum/TF 拓扑并全部转绿；清洁运行中 `/tf` publisher=3、`/tf_static` publisher=4（含唯一 `map → odom`）、`/odom/gps≈(0.04,0.18,0)`、`/odometry/filtered≈(0.04,0.18,-0.54)`，四轮 frame pair 全部存在。
- 本轮性能基线：修复后 `/odometry/filtered≈30 Hz`、`/Odometry≈20 Hz`、`/velodyne_points≈14.6 Hz`（后段有一次负载抖动）、`/joint_states≈200 Hz`；清洁运行日志未出现 EKF 更新率失败、TF 错误或持续 `No Effective Points`。
- 本轮 Web 契约验证：数组点 payload 先在回调级红灯复现 `.get` 崩溃，修复后错误批次保留旧任务且下一批合法对象点可继续处理；真实 `/web/areas` topic 运行态同样保持 `/multi_area_definer` 在线，并记录 `Web 区域数据校验失败`，随后成功提交 1 个区域。
- 本轮全量回归：新增 payload 测试、8 项 sensor-chain 测试、障碍物回归、规划器回归和 `mower_coverage` colcon 测试通过；Web socket 端口测试在获准本地环境通过。`outdoor_sim` colcon 仍被仓库既有 flake8/pep257 和离线 xmllint schema 失败阻断，未修改无关基线。
- 默认完整 profile 最终验收：HTTP `8080` 返回 200，rosbridge `9090` 启动，`/scan` publisher/subscriber 均 BEST_EFFORT，`/tf` publisher=3 且无 `odom_to_tf`，GPS/EKF 保持本地坐标；20 Hz EKF 契约通过，但长跑仍记录 FAST-LIO 无有效点及少量 EKF 阻塞提示。
- 交接尾项复验：独立 Web payload、传感器、障碍物、规划器和 `git diff --check` 全部通过；`mower_coverage` 首次沙箱运行 15/16 通过且唯一失败为 socket `PermissionError`，获准本地环境完整重跑为 16 tests / 0 errors / 0 failures。
- 本轮诊断证据：历史完整运行日志出现用户给出的 `Failed to meet update rate! Took 0.945s`，并同时存在大量 LiDAR timestamp 回退/缓冲清空；静态检查确认 FAST-LIO 20Hz 仿真输入仍配置 `scan_rate: 10`、PointCloud2 无 per-point `time`，这些是下一窗口的单变量探针候选。
- 本轮控制探针：不启动 ROS/Gazebo 的内存 harness 已验证 safe 避障状态会完全压制前进命令；无障碍状态同一控制循环会产生正向速度，说明控制器本身不是“永远不发布 cmd_vel”，需要优先检查真实 `/scan` 命中情况。
- 本轮范围：没有修改源码、参数或 launch；当前结论是诊断和下一步执行计划，不宣称 FAST-LIO 或路径执行问题已经修复。

# Summary

- Deviations count: 15（包含本轮运行环境处理、TF owner 条件化、仿真 GPS datum、Web 单向错误日志和 EKF 20 Hz 负载匹配）。
- Most likely revisit: PointCloud2 header 时间回退导致 FAST-LIO 清空缓冲，并与 safe 模式近距离扫描误停车共同造成“原地附近走不到首个目标点”。
- Edge cases found: 28（包含重复动态/静态 TF、GPS 原点错配、数组点 payload、非法几何批次、单向 Web 错误反馈、长跑点云阻塞、沙箱 TCP 限制、点云时间回退、safe 扫描误停车和原始/融合里程计契约分裂）。
- Verification status: 9 项 sensor-chain、2 项 Web payload、障碍物/规划器回归、mower_coverage 16 项 colcon 测试、三包构建、HTTP/Web socket 和默认 sensor_full 运行验收通过；outdoor_sim 仍保留既有 lint/xmllint 基线失败。
- Next session should read first: 本文件新增的 Questions for review、Verification evidence 与 Active Handoff；先做点云时间戳/`/scan`/`/cmd_vel` 实时快照和单变量 A/B，再决定 FAST-LIO、safe 避障或 odom 契约的修复顺序。

## Active Handoff（当前交接进度）

- 当前进度：路径 seam、sensor_full TF/GPS/RViz、Web payload 原子校验和默认 profile 验收已完成；本轮新增“执行器长期在起点附近、FAST-LIO 无有效点、EKF 超时”诊断，尚未修改源码，已形成下一窗口的最小验证顺序。
- 当前提交：本轮代码与交接记录已提交到本地 `HEAD`；当前分支为 `fix/web-launch-obstacle-planning`，未推送远端。
- 验证状态：9 项 sensor-chain、2 项 Web payload、障碍物与规划器回归、`mower_coverage` 16 项 colcon 测试、三包构建和获准本地 Web socket/HTTP 回归通过；`outdoor_sim` lint/xmllint 仍有既有基线失败。
- 保留状态：附加 worktree 位于 `/home/yh/mower_ws-worktrees/`，历史生成物位于 `/home/yh/mower_ws-archive/2026-07-13/`。
- 已知问题：完整高负载长跑同时出现大量 FAST-LIO `lidar loop back, clear buffer`、`No Effective Points`/`No point, skip` 和 EKF update-rate failure；20 Hz 只匹配 EKF 目标频率，尚未解决点云 header 时间回退或 CPU 负载。执行器 safe 模式还可能因真实 `/scan` 近距离命中而原地转向/倒车；原始 `/odom` 与融合 `/odometry/filtered` 的控制契约也未决。若产品需要浏览器可见的校验错误，仍需另定义响应 topic/服务契约。
- 本次实现：`hill_full` 默认关闭 `/odom→TF` 中继，`hill_ekf` 删除重复 `map→odom`、改为 `odom→camera_init`、配置仿真 GPS datum，并将仿真 EKF 频率从 30 Hz 调整为 20 Hz；`hill_sim/web_minimal` 默认中继行为保留。`multi_area_definer` 先校验整批 Web payload，再原子替换任务状态，非法批次保留旧任务。
- 当前会话进度（实时）：已读取历史运行证据并复现 safe 控制器的“持续障碍命中则无前进”行为；本轮未改源码，交接尾项 `mower_coverage` 16 项包级测试已在授权本地环境全绿。
- 下一窗口应先读：本文件的 Questions for review、Verification evidence 与 Known issues；第一阶段只采集 topic/参数/命令证据并做单变量 A/B，不要自动推送。
