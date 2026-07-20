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


def test_websocket_url_uses_the_host_serving_the_page():
    app = APP_FILE.read_text()

    assert 'window.location.hostname' in app
    assert 'ws://localhost:9090' not in app
    assert 'RTK固定解' not in app
    assert 'GNSS定位有效' in app
