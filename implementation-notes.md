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
- 本轮按实机部署计划新增 `mower_hardware`、RK3588 Docker/Compose 和 RTK 只读 profile；目标机尚未完成 SSH 盘点，因此只实现可同步的仓库侧骨架，不宣称 ARM64 镜像或串口验收通过。
- 计划默认使用标准 `nmea_navsat_driver`，但当前 WSL 未安装该运行包且没有 UM982 原始输出；保留参数化串口入口，若目标机 ARM64 包或 NMEA 格式不满足，再以可逆方式切换驱动实现。

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
- 本轮宿主机确认没有预存 ROS/Gazebo 实例；沙箱内 `ros2 topic list` 受本地 socket 权限限制，改用获准宿主机只读诊断，未启动重复实例。
- 最小 profile A/B 中 `/scan` 持续有消息但前方 ±30° 没有有效点；safe 执行器对人工 `map` 路径发布 45 次正向命令，原始 `/odom` 走到约 `x=4.4m`。不能把 safe 阈值放宽当作已证实修复。
- 完整 profile 人工路径仍能执行：`/cmd_vel` 81 次正向、1 次停止、无倒车，原始 `/odom` 走到约 `x=6.76m`；点云 header 约 20Hz 且无时间回退，FAST-LIO/EKF 也有输出，但 EKF 末位约 `(-0.003,-0.003)`，与原始 `/odom` 明显分离。
- 当前保存的 Web 区域文件规划首点约为 `(-14.601,15.011)`、距原点 `20.941m`，完整路径约 `28002` 点、边界约 `[-29.494,-32.313,84.876,51.461]`；人工短路径尚未覆盖这个真实首目标/坐标范围。
- 真实路径 safe 红灯已稳定复现：规划器输出原始 `28002` 点、执行器收到降采样 `3852` 点；`/scan` 前方累计 `7395` 个有效点中约 `99.97%` 小于 `0.25m`，safe 在 45 秒内 `cmd_vel` 正向为 0、倒车 62 次、停止 128 次。
- 同一真实路径 direct 对照转绿：`cmd_vel` 正向 71 次、倒车 0 次，原始 `/odom` 走到约 `(-4.8,5.1)`；证明规划首点和原始里程计链可执行，阻断点在 safe scan 判定。
- `/scan` 启动参数 A/B：启动时 `min_height=0.30` 仍红（前方 2206 点、正向 0），高度窗 `0.30–1.0m` 仍红（前方 2668 点、正向 0）；启动时 `min_height=1.0` 转绿（前方 0 点、正向 70 次）。运行时参数服务虽返回成功但不改变输出，不能用动态 set 做配置验收。
- 高程图定义为 30×30m、0–4m 高度起伏；启动时 `min_height=1.0` 可能过滤坡面，但也会漏掉世界内低于 1m 的灌木/岩石，不能未经产品确认成为默认安全策略。
- 用户已明确安全模式必须继续检测低于 1m 的障碍物；因此放宽绝对高度过滤不再是可接受方案，修复必须改为坡面/地面分割。
- 已在点云转换 seam 新增红灯测试 `test_hill_ground_obstacle_scan.py`；基线因缺少待实现的 `hill_ground_obstacle_scan` 模块失败，确认测试能捕获本次具体症状。
- 仅在当前 hill profile 替换通用绝对高度转换器；`nav2_sim_launch.py`/`slam_nav2_launch.py` 等旧入口未扩展，避免把仿真专项策略默默变成所有硬件 profile 的默认策略。
- 仿真 PointCloud2 含 `x,y,z,intensity,ring` 混合 datatype；Humble 的 `read_points_numpy()` 会断言失败，改用结构化 `read_points()` 只读取 XYZ，保留输入字段兼容性。
- 为满足 10 Hz 点云负载，将逐 cell `np.quantile()` 改为 NumPy 排序后按索引取下四分位；单帧基准约从 `0.265s` 降到 `0.039s`，没有引入 C++ 重构。
- 为避免新的近场盲区，最终保留 `range_min=0.20m`；车体内部回波由 `self_filter_x/y` 矩形过滤，转换异常则发布 `range_min` 停车扫描。
- 双轴审查发现当前地面模型的 fail-closed 边界不完整：只要全局平面存在，未被 `supported` 支持的 cell 仍可能被过滤；仅由分散低矮回波组成的平面也可能自举成“地面”。修复前不能宣称低矮障碍检测完备。
- 审查还发现新节点的 `numpy`/`sensor_msgs_py` 运行依赖未登记，专项回归未注册到 `outdoor_sim` 的 CTest；补齐依赖与测试注册后再提交。
- 复审进一步构造了 16 格×4 点、共 64 个、跨距约 1m 的共面 obstacle-only 回波；仅提高总点数仍会自举成地面。最终要求至少 32 个 occupied cells 且至少一半有多点 seed 支撑，并用每格真实低位回波拟合，避免把 cell 中心偏差误当成地形。
- 为覆盖更大 adversarial 平面，ground seed 还必须在 cell 内有至少 `0.02m` 的 Z 展宽；均匀共面 obstacle-only 点即使占满 32 格也不会成为地面模型。该规则对过于稀疏/无垂向地面证据的帧选择安全停车，真实硬件仍需标定。
- safe scan 复审还发现 `-Inf` 不能代表“无回波”（只有 `+Inf` 可以）；现已将 `-Inf` 与 NaN 一样判为无效并停车，回归覆盖 NaN、`+Inf` 和 `-Inf`。
- 复审指出 0.08m 容差仍会漏掉 5cm 突起；将安全容差降至 0.03m，并加入 5cm 障碍回归。该阈值仍代表测量/拟合容差，不是绝对高度过滤。
- 复审还发现执行器在 safe 模式启动、点云节点崩溃或 `/scan` 陈旧时默认继续跟踪路径；补充 0.5s scan freshness/validity 门禁，安全模式在首帧缺失、格式无效或超时期间只发布停车命令，direct 模式保持原行为。
- 当前 WSL 没有 Docker CLI，不能在开发机执行 `docker compose build`；Docker 构建明确留给 RK3588 原生 ARM64 环境。
- 现有 Web 页面把 `NavSatStatus.status` 数字误标为 RTK 固定/浮点/单点；首版改为只显示“GNSS 定位有效/无定位”，避免把 `/gps/fix` 的通用状态当成 RTK 解类型。
- ROS launch 默认会在 `~/.ros/log` 创建目录；当前沙箱该路径只读，使用 `ROS_LOG_DIR=/tmp/mower_ros_logs` 后硬件 launch 参数解析正常。
- Docker Compose 使用稳定的 `/dev/serial/by-id` 主机设备映射，并明确不映射 `can0`、不使用 `privileged`；真实设备路径、波特率和权限仍需目标机确认。
- `nmea_navsat_driver` 和 `rosbridge_server` 当前未安装在 WSL 的 ROS 环境中，但目标 Docker 镜像会在 RK3588 ARM64 上安装；本地只能做 launch 结构验证。
- 目标机改用有线网络后 SSH 已可达，但实际 OS、Docker Compose、UM982 设备路径、波特率和 NMEA 内容仍未从目标机回读。

# Questions for review

- 已处理：`/scan` QoS、`body -> gps_link` frame 契约、PointCloud2 缺失 `time`、点云输入 remap、GPS 输出 remap，以及 navsat TF 等待窗口。
- 后续性能专项仍需处理长时间高负载下 FAST-LIO `No Effective Points` / `No point, skip` 与由此造成的 EKF 阻塞；20 Hz 只解决 EKF 与 FAST-LIO 输入速率不匹配，完整长跑仍出现少量 update-rate failure，不能宣称性能专项完成。
- 已处理：`sensor_full` 的 RViz2 车体闪烁。修复后 `/tf` 仅保留 EKF、FAST-LIO、robot_state_publisher 三个 publisher，`/tf_static` 仅一个 `map → odom`，四轮 frame pair 持续发布。
- 若产品需要浏览器直接显示 Web payload 错误，需要另行确认响应 topic/服务契约；当前实现按现有单向 `/web/areas` 协议记录明确 error log，不向浏览器发送新消息。
- 新问题的下一步应先固定点云时间单调性和执行器实际停车原因，再处理 EKF 参数；不要在没有 `/scan`、`/velodyne_points` 时间戳和 `/cmd_vel` 实测数据时直接降低滤波频率或放宽安全阈值。
- 产品/架构待确认：覆盖执行与网页机器人位置应继续使用仿真原始 `/odom`，还是统一改用 EKF `/odometry/filtered`；前者便于仿真验收，后者才符合当前融合定位链，不能默默混用。
- 下一窗口需要 A/B 验证：关闭 FAST-LIO/EKF、关闭 `/scan` 或使用 direct 执行模式分别测试“控制器能否驶向首个目标点”；每次只改变一个开关并记录 `/cmd_vel`、`/odom`、`/scan` 和执行状态。
- 本轮干净短跑没有复现用户的真实停滞；下一步必须用当前 Web/区域文件生成的真实首目标重跑 A/B，并同时记录路径首点、`frame_id`、原始 `/odom` 和 `/odometry/filtered`，否则不能选择修复执行器、规划器或定位契约。
- 本轮 24–35 秒探针均未观察到 `/velodyne_points` header 回退；历史长跑的回退证据仍有效，但尚不足以证明它是本次干净人工路径停滞的原因。
- 产品决策已明确：优先保留低矮障碍安全检测并实现坡面/地面分割，不接受将仿真/坡面 profile 的检测下限提高到约 1m。
- 已决策：保守保留低矮障碍检测；仅过滤有局部地面证据且高度差在容差内的点，无法确认是地面的点 fail-closed 保留。
- 当前实现使用 8 m 局部单平面估计；仿真回归和运行态已验证，真实硬件仍需用带低矮障碍的现场点云做标定/验收，不能将本次仿真证据等同于通用安全证明。
- 路径专项的验收必须先证明完整全局路径（包括区域间 transit 和前端降采样后的 `/coverage/multi_path`）不与任一膨胀障碍物相交，再调整路径密度参数。
- 本轮路径专项已建立最小红灯：`test_multi_area_transit_avoids_next_area_obstacle`；修复前失败点为第二个区域左边界的 transit 终点进入膨胀障碍物，修复后需同时复验原始路径与降采样路径。
- 本轮审查新增红灯：`test_global_detour_avoids_all_separated_obstacles`；修复前命中 `(5.3, 1.9) → (2.7, -0.2)`，说明候选点安全不等于候选线段安全。
- 本轮新增红灯：`test_angled_obstacle_does_not_delete_area_two_coverage` 与 `test_two_areas_keep_second_area_coverage_with_obstacles`，均已在 seam 修复后通过，并复验降采样路径安全。
- 本轮建议已落实：保留每个区域已通过局部障碍检查的 `wp`，只对“前一区域末点 → 后一区域首个覆盖点”的 transit 连接做全局碰撞修复；不再对拼接后的所有区域路径统一流式调用 `_fix_obstacle_crossings()`，全局障碍物在连接前已收集。
- 实机第一阶段必须先完成 RK3588 OS/架构、Docker Compose、UM982 串口路径、波特率和原始 NMEA 盘点；在此之前不能宣称 `/gps/fix` 运行态验收。
- RTK 固定/浮点状态是否需要浏览器显示仍应通过独立 `/rtk/status` 契约确认；本轮不复用 `NavSatStatus.status` 猜测。
- `code-review` 双轴代理两次等待均未返回报告，已关闭；主代理按同一 Standards/Spec 要求人工复核当前改动，未发现需要阻断提交的硬性问题。

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
- 本轮审查：按用户 AGENTS 的 Standards 和本交接 spec 完成双轴 review；初审发现地面模型自举、低容差盲区和 safe scan fail-open，已在本轮增量修复并加入回归。后续复审需以当前工作树为准。
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
- 本轮范围：修改 hill profile 的点云→`/scan` seam、其安装/依赖、传感器契约测试，并为 safe 执行器增加首帧/有效性/新鲜度停车门禁；未改变硬件默认策略、执行器障碍停车阈值或 FAST-LIO。
- 本轮宿主机运行证据：最小 profile 与完整 profile 均用同一人工路径触发 `/multi_area/plan_and_start`；两者均有正向 `/cmd_vel` 和原始 `/odom` 位移，完整 profile 点云 stamp 无回退，未形成用户真实停滞的红灯。
- 本轮真实路径静态证据：当前区域文件的第一个覆盖点在原点约 20.9m 外，规划器发布的是降采样 `/coverage/multi_path`；需要先对这条真实路径做运行态首目标验收。
- 本轮真实路径运行证据：safe 红灯、direct 绿灯、启动高度窗口 A/B 均已在宿主机自动退出实例中完成；所有临时 checkpoint 写入 `/tmp`，源码未改，仅更新本文件。
- 本轮安全修复红灯：`python3 src/outdoor_sim/tests/test_hill_ground_obstacle_scan.py` 在实现前以 `ModuleNotFoundError` 失败；下一步先让最小地面分割函数转绿，再接入 ROS 节点。
- 本轮实现验证：4 项地面分割回归、10 项传感器契约、Python 语法检查和 `outdoor_sim` 构建通过；单实例 `/scan` 约 `12.6 Hz`，前方 ±30° 15 秒内无有效命中。
- 审查修复验证：精确复现的 24 个（8 格×3 点）和 64 个（16 格×4 点）`z=0.4m` 障碍回波全部保留；新增 5 cm 障碍、障碍-only 点云和无局部支撑点回归均通过；`outdoor_sim` CTest 已注册并通过 8 项专项测试。
- 审查修复后的包验证：`colcon build --symlink-install --packages-select outdoor_sim` 成功，安装后可枚举 `hill_ground_obstacle_scan.py`；完整 `outdoor_sim` CTest 的新 pytest 通过，剩余 flake8/pep257/xmllint 为已有基线失败。
- 审查修复后的性能基准：76,800 点合成帧地面分割中位耗时约 `0.071s`，输出 0 个平面地面障碍；仍需现场点云与真实 Web safe 长跑复验新阈值。
- 获准本地环境重跑最终版本 `mower_coverage` 后 19/19 通过；沙箱内唯一的临时 socket `PermissionError` 已确认是环境限制，不是代码回归。
- 新增 safe 执行器 freshness/scan validity 回归 3 项，和 Web payload 回归合计 5 项直接 pytest 通过；执行器只读复查未发现新的 Standards 违规。
- 新参数真实短跑：无 FAST-LIO/EKF 的 hill profile 启动成功，真实 `/scan` 发布中位约 `18.9 Hz`（8 秒观测窗口），实例已自动退出且未留下重复进程；该证据只覆盖转换发布率，不等同于完整 Web safe A/B。
- 真实坡面无障碍过滤探针（最终 32-cell/2cm-span 配置）：约 8 秒收到 161 帧，前方 ±30° 有效点 `0`、`<0.8m` 命中 `0`；实例正常清理。该证据覆盖地面误报，不覆盖真实障碍物回波。
- 本轮审查前版本真实路径 safe 绿灯：同一 Web 区域规划 `28002` 点、执行器收到 `3852` 点；35 秒内 `/cmd_vel` 前进 `305`、倒车 `0`、停止 `16`，`/scan` 前方有效命中 `0`，原始 `/odom` 从约 `(-25.77,15.46)` 到 `(2.15,15.83)`；该证据不替代当前阈值/门禁版本复验。
- 本轮审查前版本 direct 对照绿灯：清洁单实例、同一路径、同一地面过滤器；35 秒内 `/cmd_vel` 前进 `314`、倒车 `0`、停止 `14`，原始 `/odom` 从约 `(-19.91,15.46)` 到 `(8.94,15.90)`；规划器和 direct 链未改动。
- 运行中曾发现两套同名 profile 残留导致 `/scan` 双发布；已清理并用单发布者重跑，最终证据未使用重复实例数据。
- 新增 `mower_hardware` 包和 `rtk_readonly.launch.py`；`colcon build --symlink-install --packages-select mower_coverage mower_hardware` 成功，安装后的 launch 参数通过 `ros2 launch ... --show-args` 解析。
- 新增部署静态回归 5/5 通过；Compose YAML、host network、无 `privileged`、只映射 UM982 串口、无 CAN 映射和 WebSocket 主机动态地址均通过检查。
- 本轮 `mower_coverage` 独立测试为 23/24；唯一失败仍是沙箱创建临时 TCP socket 的 `PermissionError`。选定包 `colcon test` 的失败来自同一 Web socket 限制；`outdoor_sim` 既有 lint/pep257/xmllint 结果未纳入本次范围。
- 目标机尚未执行 Docker 原生构建、容器重启、`/gps/fix` 频率、UM982 NMEA、Win11 浏览器和 `/cmd_vel` 无发布者验收；这些是下一阶段现场证据。

# Summary

- Deviations count: 18（含本轮实机 Docker/RTK 部署骨架、标准 NMEA 驱动入口和 GNSS 状态显示修正）。
- Most likely revisit: UM982 实际 NMEA 输出与 ARM64 `nmea_navsat_driver` 可用性；需目标机原始串口证据，不能凭型号或截图猜测。
- Edge cases found: 45（新增 Docker 缺失、串口稳定路径、NavSatStatus 语义和 ROS 日志目录限制）。
- Verification status: 新增部署静态回归 5/5、`mower_coverage`/`mower_hardware` 构建和 launch 参数解析通过；独立测试 23/24，唯一失败为沙箱 socket 限制；目标机 Docker/UM982 运行态尚未验证。
- Next session should read first: 本文件的 Questions for review、Verification evidence 与 Active Handoff；优先执行 RK3588 SSH 盘点、原生 Docker 构建和 UM982 NMEA 只读验收。

## Active Handoff（当前交接进度）

- 当前进度：已新增独立 `mower_hardware` 包、`rtk_readonly.launch.py`、RK3588 ARM64 Docker/Compose 部署骨架，并修正 WebSocket 远程主机地址和 GNSS 状态误标；不启动仿真、规划器、执行器、CAN 或电机。
- 当前提交：本地改动待提交；当前分支为 `fix/web-launch-obstacle-planning`，不推送远端。
- 验证状态：`mower_coverage`/`mower_hardware` 构建成功，部署静态回归 5/5，launch 参数解析通过；独立测试 23/24，唯一失败是沙箱 TCP socket 权限限制。目标机原生 Docker 构建、UM982 NMEA、`/gps/fix` 和浏览器验收尚未执行。
- 保留状态：附加 worktree 位于 `/home/yh/mower_ws-worktrees/`，历史生成物位于 `/home/yh/mower_ws-archive/2026-07-13/`。
- 已知问题：完整高负载长跑仍可能出现 FAST-LIO `lidar loop back, clear buffer`、`No Effective Points`/`No point, skip` 和 EKF update-rate failure；本轮只解决 safe 的坡面误停车与 scan fail-open，不宣称点云时间回退/CPU 性能专项完成。实机侧尚无 CAN、Mid-360/IMU 驱动，第一阶段只允许 UM982 只读。
- 本次实现：`hill_full` 默认关闭 `/odom→TF` 中继，`hill_ekf` 删除重复 `map→odom`、改为 `odom→camera_init`、配置仿真 GPS datum，并将仿真 EKF 频率从 30 Hz 调整为 20 Hz；`hill_sim/web_minimal` 默认中继行为保留。`multi_area_definer` 先校验整批 Web payload，再原子替换任务状态，非法批次保留旧任务。
- 当前会话进度（实时）：仓库侧实机部署骨架已完成；WSL 已可 SSH 到 RK3588，下一步执行 OS/Docker/串口/CAN 只读盘点。目标机未完成盘点前，不配置真实 `.env`、不启动容器、不触碰电机。
- 下一步：收到 RK3588 盘点结果后确认 `UM982_HOST_DEVICE`、波特率和 ARM64 ROS 包，再在目标机原生 `docker compose build && docker compose up -d`；随后验收 `/gps/fix`、Web `8080/9090` 和 `/cmd_vel` 无发布者。不要推送未经现场验证的配置。
