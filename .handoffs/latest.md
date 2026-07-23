# Handoff: remote-capture closure and map interaction

Date: 2026-07-23
Workspace: `/home/yh/mower_ws`
Branch: `fix/web-launch-obstacle-planning`
Current HEAD: `3295acd Complete remote-capture planning and simulation safety`

## Session focus

Carry the confirmed remote-capture workflow into the next implementation window: decouple free simulation teleop from capture sampling, support sequential work-area/no-go/corridor capture, and send the complete confirmed mission to the global planner. The self-intersection recovery work is complete; the last user trial also exposed a planner-service startup configuration issue.

## Product and safety boundaries

- Preserve the legacy Leaflet manual work-area/no-go-zone workflow and its planning chain.
- Preserve raw trajectory evidence, effective geometry, drafts, explicit finish, confirmation, undo, and multi-object support.
- Closure distance `<= 0.5 m` is only the endpoint gate; it does not authorize an invalid or self-intersecting polygon.
- Do not silently close or repair geometry. A non-valid polygon remains draft and cannot be confirmed or loaded for planning.
- This work is simulation/offline only. Do not start CAN, Mid-360, hardware motion or global `/cmd_vel`; an isolated planner/executor runtime is allowed only with private `/simulation/...` command topics and no real hardware bridge.
- Simulation command output remains private under `/simulation/...`.
- A planning-only simulation may start the planner/executor nodes with `execution_mode=safe` and private plan output, but it must not start real motion or publish global `/cmd_vel`.

## Completed in this conversation

- Active `draft` sessions continue sampling and allow teleop until the user explicitly finishes again. Raw samples are not replaced by a synthetic closing point.
- Web feedback includes start point, end point, closure distance, and closure threshold.
- Route layers use `captureRoutePane` above the red robot marker; the robot marker was reduced so it does not hide the route.
- During `capturing` or `draft`, the Web preview no longer calls `map.fitBounds()` on every sample. Only transition to `ready` may perform a one-time fit; subsequent map dragging/zooming is preserved.
- Invalid geometry diagnostics now retain Shapely validity detail such as `Self-intersection[...]`; the Web distinguishes self-intersection from generic invalid geometry.
- The Web now marks the reported self-intersection with a red map marker and offers “重新采集” and “手动修正”. Manual correction submits `manual_geometry`; raw trajectory evidence is preserved.
- Manual correction mode exits on ROS disconnect or page clear, and saved drafts retain their red intersection marker.
- The Web speed control and simulation teleop path already support a maximum of `2.0 m/s`; the default remains `0.60 m/s`.
- Legacy manual-area-to-planning behavior and explicit confirmation before “载入规划” were preserved. The legacy adapter still rejects corridors, while the new `/web/mission` planner path consumes confirmed corridor geometry explicitly.
- User successfully created a work area and clicked “载入规划”. The subsequent “规划路径” failure was diagnosed as the launch profile using `with_planning_execution:=false`, so `/multi_area/plan` was intentionally absent; the old fixture process was stopped and ports `8080/9090` were released.

## Latest raw evidence

Files currently in the workspace and intentionally untracked:

- `/home/yh/mower_ws/remote_capture_debug.yaml`
- `/home/yh/mower_ws/remote_capture_active_state.json` (only 76 bytes; the ROS topic command failed and this is not useful evidence)

The YAML contains one `work-area-1` draft with `2531` raw points. The screenshot showed `563` points because sampling continued before “保存草稿”; the crossing already occurred before the 563rd sample.

Observed values:

- start `(2.2560502861, 0.6636804384) m`
- end `(2.4608881400, 0.5389187043) m`
- closure distance `0.2398416909 m`
- closure threshold `0.5 m`
- localization `GREEN`
- validity issue `Self-intersection[2.48719093867701 -1.89752687326652]`

The crossing is real in the effective trajectory, not just a closure-distance error. The earlier segment between raw samples 205–206 crosses the later segment represented by raw samples 216–221 at approximately `(2.48719094, -1.89752687) m`. The lower part of the route is hidden by the capture panel in the screenshot.

## Resolved product decision

The user confirmed that self-intersection remains fail-closed. A short turn/backtrack self-crossing is not auto-cleaned, globally simplified away, or repaired with `buffer(0)`.

The UI marks the reported intersection and provides “重新采集” and “手动修正”. Manual geometry is accepted only after finite-point and Shapely validity checks; raw evidence remains unchanged.

## Unresolved issues

- The normal manual launch profile used `with_planning_execution:=false`, so the UI can load confirmed areas but the legacy planning service is absent. Planning-only validation must explicitly start the planner stack and then decide whether that should become the simulation profile default.
- The current joystick gate is coupled to an active capture session; it cannot reposition the simulated robot between confirmed objects. The next change must separate movement permission from sampling permission.
- The new planner path consumes confirmed corridor centerline, width, endpoint IDs, bidirectionality and execution order. Invalid topology, narrow corridors, no-go envelope collisions and unsafe transit connectors fail closed; the legacy adapter continues to reject corridors rather than dropping them.
- The user confirmed the new workflow but has not authorized real-robot free teleop. Continue simulation/offline only until the control-owner, emergency-stop, timeout, and hardware boundary are explicitly confirmed.

## Verification evidence

- Host-environment full regression: `96 passed`.
- Sandbox full regression: all other tests passed; the only failure was the existing temporary TCP socket test blocked by sandbox permissions.
- `node --check` for the Web app passed.
- Python compilation, `git diff --check`, launch `--show-args`, and `colcon build --symlink-install --packages-select mower_coverage` passed.
- `mower_coverage` full pytest regression passed: `99 passed`; the focused capture/geometry/ROS adapter suite passed: `46 tests`.
- Final isolated fixture (`ROS_DOMAIN_ID=77`, `pose_topic=/trial/odom`, planning/execution disabled) produced `draft`, empty geometry, closure distance `0.141421m`, and `Self-intersection[2 2]`; saving as draft preserved all 5 raw points and the issue.
- Final Web health checks returned HTTP `200` on `8080`, rosbridge listened on `9090`, and `/cmd_vel` had no topic/publisher while `/simulation/cmd_vel` had the sole private simulation output.
- The final fixture has since been stopped; no ROS/Gazebo process or `8080/9090` listener remained at the last diagnostic check.
- No CAN, Mid-360 or real-robot motion was started. The current isolated planner/executor runtime used only `/simulation/...` command topics and was stopped after verification.
- Reproduced the user feedback: restart persistence is intentional because the remote-capture node loads `~/.local/state/mower_coverage/remote_capture_mission.yaml`; the missing capability was per-object deletion.
- Added a persisted `delete` command. The Web mission list now renders a delete button for each confirmed object/draft with confirmation; active capture disables deletion, and a work area referenced by a confirmed or draft corridor is rejected atomically.
- Deletion is also disabled while a plan is loaded, executing, or paused; the user must stop and clear the plan first so a stale path cannot survive a mission mutation.
- The Web robot marker now shows a yellow heading arrow from `/odom` orientation and displays the compass heading beside the simulated pose; invalid orientation hides the arrow.
- Movement and sampling are separated: `drive_allowed` is true only for fresh pose + `GREEN`, independent of capture session; `sampling_allowed` is true only for `capturing/draft`, and raw samples are recorded only then. After saving/confirming an object, the simulation can move to the next object without cross-object raw trajectory.
- Isolated `ROS_DOMAIN_ID=78` runtime verified idle movement through private `/simulation/cmd_vel`, no `/cmd_vel`, empty raw path while idle, raw points during capture, and next-object start after draft save. The fixture was stopped and 8080/9090 were released.
- Verified `mower_coverage` with `101 passed` outside the sandbox, focused capture/store tests, JavaScript syntax, and `git diff --check`.

Relevant test and implementation paths:

- `src/mower_coverage/mower_coverage/mission/model.py`
- `src/mower_coverage/mower_coverage/mission/capture.py`
- `src/mower_coverage/mower_coverage/mission/remote_capture_node.py`
- `src/mower_coverage/web_frontend/app.js`
- `src/mower_coverage/web_frontend/index.html`
- `src/mower_coverage/tests/test_mission_geometry.py`
- `src/mower_coverage/tests/test_remote_capture_node.py`
- `src/mower_coverage/tests/test_hardware_deployment.py`
- `src/mower_coverage/launch/remote_capture_sim.launch.py`
- `implementation-notes.md`

## Worktree cautions

The feature changes described here were committed as `3295acd` and pushed to `origin/fix/web-launch-obstacle-planning`. Only the raw debug artifacts listed above remain intentionally untracked. Do not use destructive Git commands, do not reset or clean the worktree, and do not stage those runtime artifacts. The current `implementation-notes.md` contains the consolidated recovery decision and latest verification evidence; reconcile documentation carefully rather than overwriting it.

## Current window update (2026-07-22)

- Safe execution root cause was confirmed: `remote_capture_sim.launch.py` had `with_lidar_scan:=false` while the executor defaulted to `safe`. The simulation profile now defaults the scan chain on; missing/stale/invalid scans still publish zero speed.
- Added executor path signatures so 1 Hz identical path republishing cannot reset completed progress. Added `/coverage/path_metadata`; corridor transit waypoints are excluded from coverage statistics.
- Added `/web/mission` selection. Captured missions now reach `plan_from_mission()` without legacy conversion; legacy Web drawing explicitly selects `use_legacy_areas`.
- New missions with adjacent work areas but no explicit corridor now fail closed instead of using an implicit cross-area straight line. The simulation command mux also rejects teleop/plan/output topics outside `/simulation/`.
- `/coverage/statistics` now exposes `safety_stop_reason`; Web displays whether execution is waiting for a fresh `/scan` or stopped for a nearby obstacle instead of showing only “执行中”.
- Isolated runtime `ROS_DOMAIN_ID=79`: `/scan` had one publisher and one executor subscriber; a work-area plan emitted non-zero `/simulation/plan_cmd_vel`, moved `/odom` to about `(0.79, 0.81)`, and `/cmd_vel` had no topic. A two-area corridor mission returned a successful plan with the recorded route endpoints.
- Final host regression after review: `111 passed`; `colcon build --symlink-install --packages-select mower_coverage`, Python/JS syntax checks and `git diff --check` passed. The sandbox-only full-test failure remains the existing temporary TCP socket permission error.

## Current session update (2026-07-23)

### Context and completed work

- The user reported that planning failed after confirming two work areas and then capturing the connecting corridor. Diagnosis found the persisted execution order was based on confirmation order, so the corridor was placed after both work areas.
- The corridor-order fix is complete: when its referenced work areas are adjacent, the mission store inserts the corridor between them; missing or non-adjacent references remain fail-closed. Relevant paths: `src/mower_coverage/mower_coverage/mission/store.py` and `src/mower_coverage/tests/test_mission_store.py`.
- Added regression coverage for the real confirmation sequence and for preserving existing order when corridor endpoints are non-adjacent. The order exception is documented in `docs/domain/remote-capture-context.md` and `docs/remote-capture-mission-plan.md`.
- Verification completed: focused mission tests `38 passed`; package tests `112 passed` with the known sandbox TCP-socket permission failure; Python compilation, `git diff --check`, and `colcon build --symlink-install --packages-select mower_coverage` passed.
- The user subsequently re-tested the Web flow successfully. The screenshot shows two covered work areas, an orange corridor transit segment, and the UI status `已规划`; execution has not been started in this session.
- Capture guidance is now clear: corridor endpoints need not exactly coincide with area edges; each endpoint must be inside or on the boundary of its corresponding work area. Recommend stopping 0.2–0.5 m inside. The metadata direction must match the actual capture direction; bidirectional execution does not bypass endpoint validation.

### Unresolved issues and boundaries

- Safe simulation execution still needs user verification, especially visible stopping for missing/stale `/scan` or a nearby obstacle and the resulting coverage statistics.
- A planner-service startup acknowledgement before enabling `载入规划` remains a separate, unimplemented Web contract; do not add silent retry or fallback without product confirmation.
- No CAN, global `/cmd_vel`, physical control ownership, or real-robot motion is authorized. The current scope remains offline/simulation only.
- Publication completed in `3295acd`; the known sandbox TCP socket test failure is environmental, not a new regression. The next window should verify safe simulation execution only.

### Next actions

1. Have the user run the planned mission in simulation only and observe safe execution, `/scan` gating, obstacle-stop messaging, and coverage statistics.
2. If endpoint validation reappears, inspect the saved mission payload and verify the first/last corridor samples match the configured start/end work areas.
3. Decide separately whether the Web should require an explicit planner-service readiness acknowledgement before enabling `载入规划`.
4. Keep CAN, global `/cmd_vel`, and real-hardware teleoperation out of scope until control ownership, emergency stop, timeout, and hardware acceptance are explicitly confirmed.

## Suggested skills

- `diagnosing-bugs`: continue with a fixed raw-trajectory repro and exact intersection evidence.
- `tdd`: add a behavior regression before any new self-intersection cleanup rule.
- `implementation-plan`: plan the teleop/capture seam and corridor-aware planner integration before editing.
- `implement`: implement the confirmed simulation workflow in small verified slices.
- `implementation-notes`: keep the root implementation notes and Active Handoff current.
- `interview-me`: resolve the remaining real-robot teleop safety boundary before any hardware scope.
- `code-review`: review the eventual geometry change against both safety/spec and legacy compatibility.
