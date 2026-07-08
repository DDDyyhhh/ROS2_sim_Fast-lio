#!/bin/bash
# install_deps.sh — 安装本阶段所有依赖
# 运行: bash install_deps.sh

set -e

echo "=== 安装自动割草机项目依赖 ==="
echo ""

# 1. ROS2 依赖包
echo "[1/3] 安装 ROS2 包..."
sudo apt-get install -y \
  ros-humble-robot-localization \
  ros-humble-rosbridge-suite \
  ros-humble-pointcloud-to-laserscan

# 2. Python 依赖
echo "[2/3] 安装 Python 包..."
pip3 install roslibpy  # 如果需要在 Python 中测试 WebSocket 连接

# 3. 编译工作区
echo "[3/3] 编译 ROS2 工作区..."
cd ~/mower_ws
export MAKEFLAGS="-j 8"
colcon build --symlink-install

echo ""
echo "=== 安装完成！==="
echo ""
echo "启动方式:"
echo "  source ~/mower_ws/install/setup.bash"
echo "  ros2 launch outdoor_sim hill_full.launch.py"
echo ""
echo "打开浏览器访问: http://localhost:8080"
