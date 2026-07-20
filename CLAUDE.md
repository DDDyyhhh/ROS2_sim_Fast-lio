# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Collaboration Model

- Treat user requests as potentially vague, non-technical, or incorrectly phrased; infer intent carefully, but do not blindly implement ambiguous asks.
- Work like a product manager before coding: restate the goal, identify the real user problem, define acceptance criteria, and ask targeted questions until the requirement is clear.
- Prefer small, verifiable iterations: reproduce the issue, explain the cause in plain language, implement the minimal fix, then verify with concrete commands and observed output.
- When the user reports UI/robot behavior from screenshots or browser tests, translate it into geometry/ROS/topic/test cases before changing code.
- Assume Codex and Claude Code may review the work: leave clear evidence, concise rationale, tests, and commit messages that another agent can audit.

## Repository and Workspace

- This repository is a ROS 2 Humble simulation/navigation workspace for Orange Pi 5 Plus / RK3588 ARM64.
- The Git repository and active ROS workspace root are both `/home/yh/mower_ws`.
- Source of truth is under `src/`; do not edit generated `build/`, `install/`, or `log/` artifacts.
- ROS package names are `fast_lio`, `outdoor_sim`, and `mower_coverage`; use these names with `--packages-select`.
- Current implementation handoff is `implementation-notes.md`; the remote-capture product plan is `docs/remote-capture-mission-plan.md`.
- Current ROS launch profiles are documented in `docs/operations/profiles.md`; archived overview and legacy operation documents are under `docs/archive/`.

## Common Commands

### Environment

```bash
cd /home/yh/mower_ws
source install/setup.bash
```

For ROS commands on the RK3588 target:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

For builds on the RK3588 target:

```bash
export MAKEFLAGS="-j8"
```

For headless Ignition/Gazebo on RK3588:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

### Build

Build everything:

```bash
cd /home/yh/mower_ws
export MAKEFLAGS="-j8"
colcon build --symlink-install
source install/setup.bash
```

Build selected packages:

```bash
cd /home/yh/mower_ws
colcon build --symlink-install --packages-select fast_lio outdoor_sim mower_coverage
```

Build one package:

```bash
cd /home/yh/mower_ws
colcon build --symlink-install --packages-select mower_coverage
```

### Tests

Run all colcon tests:

```bash
cd /home/yh/mower_ws
colcon test
colcon test-result --verbose
```

Run `mower_coverage` package tests:

```bash
cd /home/yh/mower_ws
colcon test --packages-select mower_coverage
colcon test-result --verbose
```

Run standalone Python regression tests directly:

```bash
cd /home/yh/mower_ws/src/mower_coverage
python3 -m pytest tests
```

Run one test file:

```bash
cd /home/yh/mower_ws/src/mower_coverage
python3 -m pytest tests/test_hill_boustrophedon_obstacles.py
python3 -m pytest tests/test_web_server.py
```

Run one test function:

```bash
cd /home/yh/mower_ws/src/mower_coverage
python3 -m pytest tests/test_web_server.py::test_http_server_allows_immediate_port_reuse
```

Syntax-check changed Python files:

```bash
python3 -m py_compile path/to/file.py
```

## Launch Commands

Integrated hill mowing stack:

```bash
cd /home/yh/mower_ws
source install/setup.bash
ros2 launch outdoor_sim hill_full.launch.py with_fastlio:=false with_ekf:=false with_lidar_scan:=false with_rosbridge:=true
```

Web frontend should then be available at:

```text
http://localhost:8080
```

rosbridge WebSocket runs on:

```text
ws://localhost:9090
```

Launch only rosbridge + static Web frontend:

```bash
cd /home/yh/mower_ws
source install/setup.bash
ros2 launch mower_coverage rosbridge_bridge.launch.py
```

Launch only hill coverage nodes:

```bash
cd /home/yh/mower_ws
source install/setup.bash
ros2 launch mower_coverage hill_coverage.launch.py
```

Launch hill simulation:

```bash
cd /home/yh/mower_ws
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
xvfb-run -a ros2 launch outdoor_sim hill_sim_launch.py
```

Launch FAST-LIO mapping:

```bash
cd /home/yh/mower_ws
source install/setup.bash
ros2 launch fast_lio mapping.launch.py use_sim_time:=true config_file:=mid360.yaml rviz:=false
```

## Runtime Verification

After launching simulation or full stack, verify core topics:

```bash
ros2 topic list
ros2 topic echo /clock --once
ros2 topic hz /velodyne_points
ros2 topic hz /imu/data
ros2 run tf2_tools view_frames
```

Verify Web frontend and rosbridge ports:

```bash
ss -tlnp | grep -E ':(8080|9090)'
curl -I http://localhost:8080/
```

Coverage planning and execution services:

```bash
ros2 service call /multi_area/plan std_srvs/srv/Trigger
ros2 service call /multi_area/plan_and_start std_srvs/srv/Trigger
ros2 service call /multi_area/status std_srvs/srv/Trigger
ros2 service call /multi_area/stop std_srvs/srv/Trigger
```

Useful coverage topics:

```bash
ros2 topic echo /coverage/multi_path --once
ros2 topic echo /coverage/statistics --once
ros2 topic echo /web/areas --once
```

## High-Level Architecture

### Packages

- `outdoor_sim`: Ignition Fortress simulation, worlds, URDF, robot spawn, ROS-GZ bridges, EKF launch, LiDAR-to-scan launch, full-stack hill launch.
- `fast_lio`: provides FAST-LIO LiDAR/IMU odometry and mapping through `mapping.launch.py`.
- `mower_coverage`: Python coverage-planning package; owns multi-area definition, hill boustrophedon planner, path executor, rosbridge/static Web frontend server, and Web UI assets.

### Integrated Hill Workflow

`outdoor_sim/launch/hill_full.launch.py` is the main integrated entry point. It composes:

1. hill simulation from `outdoor_sim`;
2. optional FAST-LIO from `fast_lio`;
3. optional EKF localization from `outdoor_sim`;
4. optional pointcloud-to-laserscan conversion;
5. optional rosbridge + Web frontend from `mower_coverage`;
6. multi-area coverage planning and execution from `mower_coverage`.

Use launch arguments to isolate subsystems:

```text
with_fastlio:=true|false
with_ekf:=true|false
with_lidar_scan:=true|false
with_rosbridge:=true|false
```

### Web Coverage Planning Data Flow

- Browser loads static Leaflet/roslibjs frontend from `src/mower_coverage/web_frontend`.
- `mower_coverage.web_server` serves the frontend on port 8080.
- `rosbridge_server` exposes WebSocket on port 9090.
- Frontend publishes drawn regions and obstacle rings as JSON to `/web/areas`.
- `multi_area_definer` converts GPS coordinates to local map coordinates and writes `~/.local/state/mower_coverage/hill_mowing_areas.yaml`.
- `/multi_area/plan` triggers `hill_boustrophedon`, which reads that area file and publishes `/coverage/multi_path`.
- `/multi_area/plan_and_start` triggers `multi_area_executor`, which follows `/coverage/multi_path` by publishing `/cmd_vel`.
- Execution progress is published on `/coverage/statistics`.

### Coverage Stack Generations

- `coverage_planning.launch.py` is the older single-area/demo stack.
- `hill_coverage.launch.py` is the current multi-area hill stack: `multi_area_definer`, `hill_boustrophedon`, `multi_area_executor`.
- Prefer `hill_coverage.launch.py` or `hill_full.launch.py` for Web UI, multi-area, and obstacle/island workflows.

### Execution Semantics

- `multi_area_executor` is a direct velocity controller using `/odom`, `/cmd_vel`, and optional `/scan`.
- It is not currently a Nav2 action client.
- `execution_mode=safe` performs simple reactive LaserScan stop/avoidance, not global replanning.
- Verify localization source and `/scan` availability before debugging execution behavior.

## Important Topics and Files

Simulation and localization topics:

```text
/clock
/velodyne_points
/imu/data
/odom
/gps/fix
/joint_states
/cmd_vel
/scan
```

Coverage topics and services:

```text
/web/areas
/coverage/multi_path
/coverage/statistics
/multi_area/plan
/multi_area/plan_and_start
/multi_area/status
/multi_area/stop
/multi_area/pause
/multi_area/resume
```

Runtime state files outside the repository:

```text
~/.local/state/mower_coverage/hill_mowing_areas.yaml
~/.local/state/mower_coverage/hill_coverage_checkpoint.json
```

If planning behavior looks stale after UI changes, inspect or remove these runtime files.

## RK3588 / Ignition / ROS 2 Rules

- Use ROS 2 Humble on Ubuntu 22.04.
- Prefer CycloneDDS: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
- Cap build parallelism with `MAKEFLAGS="-j8"` on RK3588.
- Use Ignition Fortress / Gazebo Sim 6.x with SDF 1.8.
- Do not use SDF `<heightmap>` on RK3588/Mali; it can trigger Ogre shader segfaults. Use flat box terrain and primitive obstacles.
- Sensors belong inside `<link>` elements as `<sensor>` children, not in `<gazebo>` extension blocks.
- Each sensor should define `ignition_frame_id` and an Ignition topic.
- For headless simulation, set `LIBGL_ALWAYS_SOFTWARE=1` and use `xvfb-run -a`.

## ROS 2 Development Rules

- For C++ ROS nodes, use C++17 and inherit from `rclcpp::Node`.
- Store ROS publishers/subscriptions/timers in smart pointers; avoid raw pointers for ROS handles.
- Use `ament_target_dependencies` instead of manually hard-coded ROS include/library paths.
- Use `RCLCPP_*` logging, not `std::cout`/`printf`, in ROS nodes.
- High-rate sensor topics such as LiDAR, IMU, PointCloud2, LaserScan, and images should use BEST_EFFORT QoS.
- Command topics such as `/cmd_vel` should be RELIABLE.
- TF and `/tf_static` should remain reliable/transient-local as appropriate.
- Catch `tf2::TransformException` and validate external sensor data before dereferencing or indexing.
- For xacro-generated `robot_description` in ROS 2 Humble launch files, wrap XML with `ParameterValue(..., value_type=str)`.
- Spawn robots with `ros_gz_sim create`, not deprecated `gazebo_ros spawn_entity.py`.

## Gotchas

- Existing docs may be partially stale around frame names (`body` vs `base_link`), LiDAR line count (32 vs 64), and FAST-LIO config names (`mid360.yaml` vs `mid360_sim.yaml`). Inspect current launch/config/URDF before changing these.
- The `fast_lio` upstream README includes Livox-specific instructions, but local `package.xml` notes `livox_ros_driver2` was removed for PointCloud2 mode. Prefer local launch/config behavior over upstream Livox instructions.
- `rosbridge_bridge.launch.py` starts both `rosbridge_websocket` and `web_server`; this is not a Node/React/Vite frontend.
- If `localhost:8080` times out, check `ss -tlnp | grep 8080` and ROS logs for `Address already in use`.
- Web frontend GPS conversion has a hard-coded origin in both frontend JavaScript and `multi_area_definer`; keep them synchronized if changing map origin.
- Path planning obstacle regressions should be captured in `src/mower_coverage/tests/test_hill_boustrophedon_obstacles.py`.
- Web server port regressions should be captured in `src/mower_coverage/tests/test_web_server.py`.
