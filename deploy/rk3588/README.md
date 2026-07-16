# RK3588 UM982 只读部署

这是第一阶段实机 profile。它只启动 UM982 NMEA 读取、`/gps/fix`、rosbridge
和 Web 页面，不启动仿真、FAST-LIO、规划器、执行器、CAN 或电机控制。

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

docker compose config
docker compose build
docker compose up -d
```

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
  'source /opt/ros/humble/setup.bash && source /opt/mower_ws/install/setup.bash && ros2 topic info /cmd_vel -v'
```

Win11 浏览器访问 `http://RK3588_IP:8080`。页面内 rosbridge 应连接到
同一台 RK3588 的 `9090` 端口。

本阶段如果没有有效 NMEA，不要猜测串口参数；停止容器后用 UM982 厂商工具
确认输出协议和波特率，再重新配置 `.env`。
