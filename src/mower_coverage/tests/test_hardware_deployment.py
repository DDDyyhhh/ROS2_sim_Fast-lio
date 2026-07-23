from pathlib import Path
import ast


REPO_ROOT = Path(__file__).resolve().parents[3]
DEPLOY_DIR = REPO_ROOT / 'deploy' / 'rk3588'
ENTRYPOINT_FILE = DEPLOY_DIR / 'entrypoint.sh'
LAUNCH_FILE = (
    REPO_ROOT / 'src' / 'mower_hardware' / 'launch'
    / 'rtk_readonly.launch.py'
)
APP_FILE = (
    REPO_ROOT / 'src' / 'mower_coverage' / 'web_frontend' / 'app.js'
)


def test_rk3588_deployment_has_required_files():
    for name in ('Dockerfile', 'compose.yaml', '.env.example', 'entrypoint.sh',
                 'README.md'):
        assert (DEPLOY_DIR / name).is_file()


def test_hardware_image_excludes_simulation_runtime():
    dockerfile = (DEPLOY_DIR / 'Dockerfile').read_text()
    dockerignore = (REPO_ROOT / '.dockerignore').read_text()

    assert 'ARG ROS_BASE_IMAGE=ros:humble-ros-base-jammy' in dockerfile
    assert 'rtk_ntrip_node = mower_hardware.rtk_ntrip_node:main' in (
        REPO_ROOT / 'src' / 'mower_hardware' / 'setup.py'
    ).read_text()
    assert 'ros-humble-nmea-navsat-driver' not in dockerfile
    assert 'ros-humble-rosbridge-server' in dockerfile
    assert '--packages-select mower_coverage mower_hardware' in dockerfile
    assert 'outdoor_sim' not in dockerfile
    assert 'fast_lio' not in dockerfile
    assert '*' in dockerignore
    assert '!src/mower_coverage/**' in dockerignore
    assert '!src/mower_hardware/**' in dockerignore
    assert '!deploy/rk3588/entrypoint.sh' in dockerignore


def test_entrypoint_does_not_apply_nounset_to_ros_setup():
    entrypoint = ENTRYPOINT_FILE.read_text()

    assert 'set -e -o pipefail' in entrypoint
    assert 'set -euo pipefail' not in entrypoint


def test_compose_maps_only_the_rtk_device():
    compose = (DEPLOY_DIR / 'compose.yaml').read_text()

    assert 'ROS_BASE_IMAGE' in compose
    assert 'network_mode: host' in compose
    assert 'UM982_HOST_DEVICE' in compose
    assert 'UM982_CONTAINER_DEVICE' in compose
    assert 'can0' not in compose
    assert 'privileged: true' not in compose
    assert 'no-new-privileges:true' in compose


def test_rtk_launch_is_read_only_and_uses_canonical_fix_contract():
    launch = LAUNCH_FILE.read_text()

    ast.parse(launch)
    assert "package='mower_hardware'" in launch
    assert "executable='rtk_ntrip_node'" in launch
    assert 'nmea_navsat_driver' not in launch
    assert "'caster_host': caster_host" in launch
    assert "'mountpoint': mountpoint" in launch
    assert "'correction_timeout': correction_timeout" in launch
    assert "'frame_id': frame_id" in launch
    assert "'use_sim_time': False" in launch
    assert "'/cmd_vel'" not in launch
    assert "'rtk_link'" in launch
    assert "default_value=_env('RTK_FIX_TOPIC', '/rtk/gps/fix')" in launch


def test_simulation_profile_publishes_an_explicit_simulated_rtk_stream():
    launch_file = (
        REPO_ROOT / 'src' / 'mower_coverage' / 'launch'
        / 'remote_capture_sim.launch.py'
    )
    launch = launch_file.read_text()

    ast.parse(launch)
    assert "executable='simulation_rtk'" in launch
    assert "default_value='true'" in launch
    assert "'/rtk/gps/fix'" in launch
    assert "'/rtk/status'" in launch
    assert "'cmd_vel_topic': simulation_cmd_vel_topic" in launch
    assert "'output_topic': teleop_cmd_vel_topic" in launch
    assert "executable='simulation_cmd_mux'" in launch
    assert "default_value='false'" in launch
    assert "default_value='/simulation/plan_cmd_vel'" in launch
    assert "'/simulation/cmd_vel'" in launch
    assert "'initial_health_state': 'RED'" in launch
    assert "'max_linear_speed': 2.0" in launch


def test_websocket_url_uses_the_host_serving_the_page():
    app = APP_FILE.read_text()

    assert 'window.location.hostname' in app
    assert 'ws://localhost:9090' not in app
    assert 'RTK固定解' not in app
    assert '仿真GNSS' in app
    assert 'RTK天线' in app


def test_web_teleop_uses_virtual_joystick_with_speed_control():
    html = (
        REPO_ROOT / 'src' / 'mower_coverage' / 'web_frontend'
        / 'index.html'
    ).read_text()
    app = APP_FILE.read_text()

    assert 'id="teleop-joystick"' in html
    assert 'id="teleop-speed"' in html
    assert 'max="2.00"' in html
    assert "name: '/teleop/cmd_vel'" in app
    assert 'JOYSTICK_DEADZONE' in app
    assert 'setPointerCapture' in app
    assert 'lostpointercapture' in app
    assert 'publishTeleop(0, 0)' in app


def test_remote_capture_geometry_fits_map_and_hides_corridor_fields():
    html = (
        REPO_ROOT / 'src' / 'mower_coverage' / 'web_frontend'
        / 'index.html'
    ).read_text()
    app = APP_FILE.read_text()
    start = app.index('function renderMissionItems()')
    end = app.index('/**\n * 将局部坐标', start)
    render_function = app[start:end]

    assert '.capture-row[hidden]' in html
    assert 'map.fitBounds' in render_function


def test_remote_capture_has_explicit_planning_handoff():
    app = APP_FILE.read_text()
    connection_start = app.index("state.ros.on('connection'")
    connection_end = app.index("state.ros.on('close'", connection_start)
    connection_handler = app[connection_start:connection_end]

    assert 'function loadCaptureMissionForPlanning()' in app
    assert 'btn-capture-load-plan' in app
    assert 'updateSendButton()' in connection_handler


def test_capture_preview_keeps_active_and_draft_routes_visible():
    app = APP_FILE.read_text()

    assert 'function renderCapturePreview()' in app
    assert 'active.raw_trajectory' in app
    assert 'item.raw_trajectory' in app
    assert '当前没有正在采集的对象' in app
    assert "if (!active)" in app[app.index('function captureCancel()'):]
    assert 'captureState.last_error' in app
    assert "'btn-capture-cancel', !state.connected" in app
    assert "map.createPane('captureRoutePane')" in app
    assert 'robot-heading-icon' in app
    assert 'robot-heading-arrow' in app
    assert 'function quaternionToHeadingDegrees' in app
    assert '轨迹未闭合，请回到起点附近' in app


def test_capture_feedback_exposes_endpoints_and_closure_threshold():
    html = (
        REPO_ROOT / 'src' / 'mower_coverage' / 'web_frontend'
        / 'index.html'
    ).read_text()
    app = APP_FILE.read_text()

    assert 'id="capture-closure-feedback"' in html
    assert 'activeState === \'capturing\' || activeState === \'draft\'' in app
    assert 'start_point' in app
    assert 'end_point' in app
    assert 'closure_distance' in app
    assert 'closure_tolerance' in app
    assert '起点' in app
    assert '终点' in app
    assert '闭合阈值' in app
    assert 'L.circleMarker' in app


def test_live_capture_preview_does_not_recenter_the_map_on_each_sample():
    app = APP_FILE.read_text()
    start = app.index('function renderCapturePreview()')
    end = app.index('/**\n * 将局部坐标', start)
    render_function = app[start:end]
    normalized = ' '.join(render_function.split())

    assert "active.state === 'ready'" in render_function
    assert (
        "if (active.state === 'ready' && "
        "signature !== state._capturePreviewSignature)" in normalized
    )


def test_capture_failure_and_confirmation_handoff_are_explicit():
    app = APP_FILE.read_text()

    assert '轨迹不构成有效多边形' in app
    assert '请点击“确认对象”后载入规划' in app
