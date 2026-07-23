# Deviations

- **采集与确认门禁（2026-07-22）**：原计划/产品期望 `finish` 后直接载入规划；实际保留 `ready → confirm → load`，因为规划只消费已确认对象。`finish` 和 `confirm` 都重新检查定位健康，RED 时保留 raw 轨迹、清空有效几何并退回 draft；只有产品明确改门禁时才需要重审。
- **继续采集**：原先误以为需要改领域状态机；实际 `CaptureSession` 已能保存未闭合 draft，修复范围收敛为 timer/drive gate 纳入 draft，不改变状态名和自动补点规则。
- **任务模型与旧 ROS 适配**：旧区域链不能无损表达全局禁区和 corridor，因此新增显式 loader/adapter；禁区挂到作业区约束，corridor 直接拒绝，避免静默丢语义。通道宽度先采用 `max(robot_length, robot_width) + 2 × safety_margin`，缺少真实机器人包络时 fail-closed，不猜实机默认值。
- **Web 输入错误**：计划没有定义新的响应契约，沿用单向 `/web/areas` topic，通过后端 error log 报错；非法批次原子拒绝并保留旧状态，不新增 response topic 或网络调用。
- **区域间规划**：原计划只改 `plan_from_areas()` seam；测试暴露全局障碍物、候选线段和失败终点问题，因此改为只对“前一区域末点 → 后一区域首个覆盖点”的 transit 做逐段碰撞修复，无法到达首个覆盖点就 fail-closed，不发布不完整路径，保留既有接口签名。
- **坡面点云转换**：不能用提高 `min_height` 到约 1m 的方式消除坡面误报，因为会漏掉低矮障碍；仅在 hill profile 使用局部地面/支持证据和 0.03m 容差，旧入口不扩展为默认策略。实现同时补齐依赖、CTest 注册和 0.5s safe scan freshness/validity 门禁。
- **传感器融合**：运行态发现了计划未列出的点云 remap、EKF frame、GPS 输出 remap、datum 和 TF owner 问题，均纳入同一最小修复；EKF 频率调整到实测约 20Hz，navsat 增加 0.2s TF 等待，不改变融合变量或硬件默认策略。
- **RTK 串口接入**：原计划默认使用 `nmea_navsat_driver`，实际改用自有 `rtk_ntrip_node`，让 RTCM 写入和 NMEA 读取共享唯一串口 owner，避免多进程抢占；凭据只从 `CORS_USER`/`CORS_PASS` 环境变量读取，不进入代码、日志或 notes。
- **部署与环境**：WSL 不承担 ARM64 Docker 构建，目标机使用可选 `ROS_BASE_IMAGE` 和 allowlist `.dockerignore`；入口脚本去掉对 ROS setup 不兼容的 `set -u`。ROS/Gazebo 运行态诊断使用可写 `/tmp` 日志和干净实例，沙箱网络/权限错误不作为代码结论。
- **CAN 范围**：资料不足时只做离线协议解码、回放和安全审计；不启用 `can0/can1`、不发送控制帧、不接管 `/cmd_vel`。物理控制权、急停、看门狗和超时行为需人工确认后另立范围。
- **自交恢复（2026-07-22）**：原先待定的短转弯自交不做自动清理；按产品确认继续 fail-closed，前端只标出交点并提供重新采集/手动修正，手动几何仍必须经过 Shapely 有效性校验。
- **指定对象删除（2026-07-22）**：重启后恢复 YAML 是既定持久化行为，实际缺口是 Web 没有删除入口；新增 `/mission/capture/command` 的 `delete` action，删除会同步持久化，活动采集时拒绝，作业区被 corridor 引用时拒绝而不级联静默删除。
- **移动与采样双门控（2026-07-22）**：原实现把 `drive_allowed` 绑定到活动采集会话；实际连续采集需要先移动到下一个边界，因此改为 `GREEN + fresh pose` 只控制仿真移动，新增 `sampling_allowed` 仅允许 `capturing/draft` 记录 raw trajectory。

# Discovered edge cases

- **采集状态与几何**：末点距起点 `≤0.5m` 不保证 Polygon 有效，轨迹自交或面积退化仍必须留在 `draft`；非闭合 `finish` 只暂存 draft，回到阈值内不会自动生成 geometry。raw 端点和有效几何必须始终分开保存。
- **任务输入原子性**：点必须是 `{x,y,z}` 对象，坐标必须有限，名称必须是字符串；order 中的 `null`、字符串或不可哈希项、`objects` 与旧 `areas` 同时出现都必须拒绝，不能让旧状态被部分替换或让异常穿透 ROS/Web 回调。
- **任务语义**：全局禁区可以位于作业区原始边界内，但有效覆盖必须扣除禁区；禁区不进入执行 order，corridor 必须连接两个作业区且其通行包络不能穿越禁区。现有 `inner_rings` 同时承载内岛/禁区/障碍物，不能直接当作新 `no_go_zones` 模型。
- **Transit 碰撞**：障碍物膨胀后可能延伸到下一个区域外，候选点安全也不代表候选线段安全；浮点误差可能把扫描端点推入障碍物。跨区连接必须逐段检查，并验证绕障结果确实到达首个覆盖点；原始路径和降采样后的 `/coverage/multi_path` 都要复验。
- **点云与融合链**：仿真 PointCloud2 为混合字段 `x,y,z,intensity,ring` 且可能没有 per-point `time`，不能假设 `read_points_numpy()` 或时间戳完整。QoS、`body`/`base_link`、GPS 输出 topic、唯一 `odom → body` owner 和仿真 datum 任一不一致，都可能表现为 `/scan` 无消息、TF 闪烁或 EKF 发散。
- **长跑时间负载**：`clock_filter.py` 只保证 `/clock` 单调，不会修正点云 header；FAST-LIO 仍可能因旧帧/时间回退清空 buffer，出现 `No point`、`No Effective Points` 和 EKF update-rate failure。20Hz 频率匹配不能单独证明融合链健康，当前仍需单变量复现。
- **地面分割与安全扫描**：障碍物-only 点云可能自举成平面，因此地面 seed 需有足够 occupied cells、多点支撑和 cell 内 Z 展宽；无法证明是地面的点必须保留并停车。NaN、`-Inf` 和无效/陈旧/首帧缺失的 `/scan` 都按不安全处理，safe 模式只发停车命令。
- **RTK 串口与新鲜度**：`ttyUSB` 编号会因重枚举变化，必须使用 `/dev/serial/by-id`；NTRIP TCP 保持 `ESTAB` 不代表有 RTCM，需同时看 `rtcm_bytes`、`last_rtcm_age_s` 和 `corrections_fresh`。探针、RTK 节点和其他串口程序不能并行打开 UM982。
- **RTK 验收口径**：首次 `quality=4` 不等于已收敛；固定解应单独统计，300秒窗口的最大偏差也不能直接等同于每个样本都在1cm内。对外使用“厘米量级静态重复性”，不宣称绝对精度；仿真 RTK 也不得冒充真实天线。
- **运行环境**：ROS Humble setup 脚本不是 nounset-safe；Compose 检查应使用 `config -q`，避免把环境变量渲染到终端。多个 profile/残留实例会造成 `/scan` 双发布或错误诊断，运行态验证前必须确认单实例和单 publisher。
- **自交恢复生命周期**：手动绘制模式必须在 ROS 断开或页面清除时退出；失败轨迹保存为草稿后仍需在草稿图层保留交点标记，避免恢复入口和诊断证据消失。
- **任务对象删除**：对象列表的删除按钮必须只发送对象 ID，并通过 ROS store 修改持久化任务；删除作业区要同步移除执行顺序，且任何已确认或草稿 corridor 引用都必须让删除原子失败。
- **规划状态与删除**：已载入、执行中或暂停中的规划禁止修改任务对象；Web 删除按钮在这些状态禁用并要求先停止/清除规划，避免旧路径继续留在规划器或地图上。
- **方向显示坐标**：Web 车头箭头以 `/odom` 四元数转换为罗盘角度，约定 ROS yaw=0 的 +x 为东、+y 为北；无效四元数隐藏箭头并显示方向未知，不伪造朝向。
- **通道执行顺序（2026-07-23）**：用户先确认两个作业区、后确认 corridor 时，旧逻辑把 corridor 追加到 order 末尾；现在仅在其两个端点作业区已相邻时插入中间，端点缺失或不相邻仍保留 fail-closed。

# Questions for review

## 产品与架构

- RTK 非 Fixed 时是否允许任务继续？建议只允许 Mid-360 在短暂且受健康检查约束的降级窗口内维持局部运动；不得用它重新定义全局边界、corridor 或跨区定位，超时即暂停。
- 覆盖执行和网页位置应继续使用原始 `/odom`，还是统一切到 `/odometry/filtered`？当前两者契约仍分裂，不能默默混用。
- 是否需要为 Web payload 错误增加 response topic/服务？在该契约确认前，继续只记录明确 error log。
- 遥控采集的 App/Web 流程是否正式进入运动实现范围？若进入，必须同时确定控制仲裁、遥控接管、急停和 `/cmd_vel` owner，不能只改 UI。

## 下一阶段验证

- 用当前 Web/区域文件的真实首目标做单变量 A/B：分别关闭 FAST-LIO/EKF、关闭 `/scan` 或使用 direct 模式，并同步记录首点、`frame_id`、`/cmd_vel`、原始 `/odom`、`/odometry/filtered`、`/scan` 和停车原因。此前短跑没有复现用户的真实停滞，尚不足以选择控制器、规划器或定位修复方向。
- 对当前 hill profile 做真实低矮障碍/坡面点云标定，并完成带 safe 门禁的长跑；仿真已验证地面误报抑制，不能当作实机安全证明。
- 对完整全局路径和前端降采样结果做现场或代表性数据碰撞验收；合成 transit/分离障碍物回归已通过，但不能替代真实场景。
- CAN 需要补充 DBC/协议手册/下位机代码或安全 `candump -L`，并由人工确认总线速率、控制权、心跳/校验、看门狗、急停和超时默认动作；在此之前保持只读边界。

## 已确认的安全边界

- `rtk_readonly` 不启用 `can0/can1`、不发送 CAN、不接管 `/cmd_vel`；仿真遥控使用 `/simulation/cmd_vel`，全局 `/cmd_vel` 不应出现 publisher。
- 仿真 `GREEN` 只表示 `simulation_local_odom` 简化健康契约，不等同于真实 map/Mid-360/IMU/点云健康，也不能授权实机运动。

# Verification evidence

- 截至 2026-07-21/22，获准本地环境记录为 `mower_coverage` 78 passed、`mower_hardware` 23 passed；三包 `colcon build --symlink-install --packages-select mower_coverage outdoor_sim mower_hardware`、Python/JS 语法检查和 `git diff --check` 通过。沙箱内 Web socket 的 `PermissionError` 已确认是环境限制，不能当作回归。
- 任务模型/旧 YAML、采集状态、定位健康、Web payload、障碍物 transit、分离障碍物和地面分割均有红灯转绿的回归；`outdoor_sim` 仍有仓库既有 flake8/pep257/离线 xmllint 基线问题，专项测试通过不等于全包质量门禁清零。
- 隔离仿真运行（`ROS_DOMAIN_ID=77`）已验证 HTTP `8080=200`、rosbridge `9090`、`/rtk/status=RTK_FIXED` 且 `source=simulation`、`global_position_trusted=true`；WebSocket 收到两个作业区、一个禁区和一个 corridor，drafts 为空，禁区不入 order。
- 正式 RTK 节点已验证单串口运行、容器重启恢复和 CORS 断流恢复。最新 300.1s Fixed-only 记录为 `VALID_GGA=300`、`INVALID_GGA=0`、quality=4 全覆盖；Fixed-only 标准差 east `0.008m`、north `0.010m`、radial `0.008m`，相对首个 Fixed 最大偏差 `0.112m`，因此只报告厘米量级静态重复性。
- CORS 断流时 `corrections_fresh=false`、`global_position_trusted=false`，恢复后重新进入 `RTK_FIXED`；凭据只通过环境变量提供，未写入仓库。现场仍不得让 NTRIP 探针与 ROS RTK 节点同时打开 UM982。
- 全部运行态证据均未启用 CAN、未发送控制帧、未接入规划器/执行器，也未进行实机运动；所有临时日志和 checkpoint 应继续放在 `/tmp` 或目标机临时目录。
- 2026-07-22 自交恢复回归：`mower_coverage` 全量 `99 passed`，`colcon build --symlink-install --packages-select mower_coverage` 通过；隔离域 `ROS_DOMAIN_ID=77` 的固定 5 点轨迹得到 `draft`、空 geometry、`Self-intersection[2 2]`，保存草稿后仍保留 5 个 raw 点和诊断。
- 2026-07-22 试用实例：HTTP `8080=200`、rosbridge `9090` 可用，最终环境保持 active 自交 draft；`/cmd_vel` 不存在，唯一仿真运动输出为 `/simulation/cmd_vel`，未启动规划执行。
- 2026-07-22 指定删除回归：ROS command callback 的删除/重载场景、corridor 依赖原子拒绝测试通过；`mower_coverage` 全量 `101 passed`（非沙箱），`node --check`、聚焦 unittest 和 `git diff --check` 通过。沙箱全量唯一失败仍是已知临时 TCP socket 权限限制。
- 2026-07-22 移动/采样与朝向回归：聚焦测试 `13 passed`，完整 `mower_coverage` `103 passed`（非沙箱），构建、`node --check` 和 `git diff --check` 通过；隔离 `ROS_DOMAIN_ID=78` 仿真验证空闲状态 `drive_allowed=true/sampling_allowed=false`，私有摇杆移动到约 `(0.88, 0.16)` 且 raw path 为空，开始对象后才出现 raw 点，保存 draft 后可开始下一个禁区。关停时 ros_gz_bridge 出现 SIGINT `exit -6`，但进程和 8080/9090 均释放。

# Summary

- Deviations count: 13 个合并主题；历史中重复的阶段性探针和环境细节已折叠，安全/契约决策保留。
- Most likely revisit: CAN 控制权与安全机制；没有协议和人工安全确认前，不应进入物理控制。
- Edge cases found: 13 个合并主题；最关键的是自交恢复生命周期、跨区 transit 碰撞、移动/采样分离、任务对象删除依赖、地面分割 fail-closed、scan 新鲜度和 RTCM 新鲜度。
- Verification status: 本次 `mower_coverage` 全量 `103/103`（非沙箱）、构建和静态检查通过；移动/采样隔离、车头角度转换、删除对象持久化重载和 corridor 依赖门禁已验证，RTK 300 秒 Fixed-only 和断流恢复证据已留存。
- Next session: 让用户在 Web 中验证车头箭头和连续采集完整流程；继续保持 CAN、`/cmd_vel` 和实机运动关闭。

## 2026-07-22 本轮执行与通道规划

- Deviations: 安全仿真 profile 将 `with_lidar_scan` 默认改为开启；`safe` 仍允许显式关闭扫描，但缺失/过期扫描继续停车，不改成 `direct` 绕过门禁。
- Deviations: 新增 `/web/mission` 规划选择 seam，采集任务不再降级为旧 `/web/areas`；旧手工区域仍显式选择 legacy adapter，corridor 不会被静默丢弃。
- Deviations: remote-capture launch 在解析参数时拒绝任何非 `/simulation/` 的 teleop/plan/output topic，simulation mux 节点也做同一层校验；旧 `hill_coverage` 入口的全局默认值保持兼容，不由本 profile 使用。
- Discovered edge cases: 规划器降采样会损失弯曲 corridor 中间点，因此中心线点被标记为必须保留；新增 `/coverage/path_metadata`，执行器按 coverage/transit 区分覆盖率。
- Discovered edge cases: safe 模式因无障碍或无新鲜扫描停车时，原 Web 状态只显示“执行中”；统计消息现在带 `safety_stop_reason`，前端明确显示等待扫描或障碍物停车。
- Verification: 隔离 ROS domain `79` 中 `/scan` 有唯一发布者且执行器订阅成功；规划执行发布非零 `/simulation/plan_cmd_vel`，仿真 `/odom` 从原点移动到约 `(0.79, 0.81)`，`/cmd_vel` 无话题。
- Verification: 包含 `area-1 → corridor-1 → area-2` 的任务返回 `规划完成: 2 个作业区 ...（含通道）`，发布路径保留 corridor 端点；所有运行均未启动 CAN、全局 `/cmd_vel` 或实机运动。
- Summary: 本轮新增执行器路径重发、safe profile、mission planner、corridor 方向和路径元数据回归，并拒绝新任务跨作业区的隐式直线连接；`mower_coverage` 主机全量 `111 passed`，构建、语法和 diff 检查通过。下一次先读本节与 `.handoffs/latest.md`，再做 Web 试用。

## 2026-07-23 本轮通道顺序修复

- Deviation: 产品确认允许“先两个作业区、后通道”的采集顺序；未重排已有作业区，只在引用的两个作业区当前相邻时插入 corridor，避免静默改变既有执行顺序。
- Verification: 新增真实 `MissionCaptureStore` 回归，覆盖作业区 1 → 作业区 2 → corridor 的确认链；任务 order 自动变为 `area-1 → corridor-1 → area-2` 且 `validate_mission()` 通过。
- Verification: 任务存储、几何、规划定向测试 `38 passed`；`mower_coverage` 全量 `112 passed, 1 sandbox PermissionError`，后者为已有临时 TCP socket 权限限制。
- Summary: Deviations count: 14 个合并主题；本轮最可能复查的是非相邻作业区引用 corridor 的产品排序交互。
- Summary: Edge cases: corridor 端点缺失/不相邻不自动重排；下一次先读本节、`.handoffs/latest.md` 和 `implementation-notes.md`，再做 Web 现场回归。
