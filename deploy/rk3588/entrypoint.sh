#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source /opt/mower_ws/install/setup.bash

exec "$@"
