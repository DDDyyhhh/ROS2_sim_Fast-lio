# RK3588 UM982 只读部署

这是第一阶段实机 profile。它只启动单一 `rtk_ntrip_node`：该节点独占 UM982
串口，同时读取 NMEA、向 CORS 写入 RTCM，并发布 `/gps/fix`、`/rtk/status`、
`/rtk/nmea`。它不启动仿真、FAST-LIO、规划器、执行器、CAN 或电机控制。

`/gps/fix` 的 `NavSatStatus` 只表示 GNSS 是否有有效 fix；RTK Fixed/Float
必须读取 `/rtk/status` 的 `solution`、`ntrip` 和
`global_position_trusted`，不能从 `NavSatStatus.status` 推断。

## 目标机准备

在 RK3588 本地终端确认：

```bash
uname -a
cat /etc/os-release
dpkg --print-architecture
ip -br addr
ls -l /dev/serial/by-id/ 2>/dev/null || true
```

安装并启用 SSH 和 Docker Engine。确认以下命令都可用：

```bash
ssh -V
docker --version
docker compose version
```

如果 `docker compose` 不存在，安装 Docker Compose plugin；不要用旧的
`docker-compose` 代替本部署文件。

## WSL 同步和构建

WSL 是源码和 SSH 的唯一写入入口。从 WSL 执行以下同步命令；它不会同步
生成的 ROS 产物、Git 元数据或目标机私有 `.env`：

```bash
cd /home/yh/mower_ws
rsync -az \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  --exclude '.git/' \
  --exclude 'deploy/rk3588/.env' \
  ./ 用户名@RK3588_IP:~/mower_ws/
```

然后在 RK3588 上执行：

```bash
cd ~/mower_ws/deploy/rk3588
cp .env.example .env
chmod 600 .env
# 编辑 .env，填写真实的 UM982_HOST_DEVICE 和确认过的 UM982_BAUD
# 在当前 shell 中通过环境变量提供 CORS 凭据，不要写入仓库文件：
export CORS_USER='你的账号'
read -r -s CORS_PASS
export CORS_PASS

docker compose config -q
docker compose build
docker compose up -d
```

当前目标网络无法连接 Docker Hub，`.env.example` 默认使用可达的
`docker.m.daocloud.io` 镜像前缀；如果目标网络可访问官方 registry，可将
`ROS_BASE_IMAGE` 改为 `ros:humble-ros-base-jammy`。

推荐从 WSL 同步时排除 `build/`、`install/`、`log/`，让目标机原生构建
ARM64 镜像。

## 验证

```bash
docker compose ps
docker compose logs --tail=100 mower_rtk

docker exec mower-rkt bash -lc \
  'source /opt/ros/humble/setup.bash && source /opt/mower_ws/install/setup.bash && ros2 topic list'

docker exec mower-rkt bash -lc \
  'source /opt/ros/humble/setup.bash && source /opt/mower_ws/install/setup.bash && ros2 topic echo /gps/fix --once'

docker exec mower-rkt bash -lc \
  'source /opt/ros/humble/setup.bash && source /opt/mower_ws/install/setup.bash && ros2 topic echo /rtk/status --once'

docker exec mower-rkt bash -lc \
  'source /opt/ros/humble/setup.bash && source /opt/mower_ws/install/setup.bash && ros2 topic info /cmd_vel -v'
```

Win11 浏览器访问 `http://RK3588_IP:8080`。页面内 rosbridge 应连接到
同一台 RK3588 的 `9090` 端口。

本阶段如果没有有效 NMEA，不要猜测串口参数；停止容器后用 UM982 厂商工具
确认输出协议和波特率，再重新配置 `.env`。
