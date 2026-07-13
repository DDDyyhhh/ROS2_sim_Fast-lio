#!/usr/bin/env python3
"""
web_server.py — Web 前端 HTTP 服务器 ROS2 节点

启动一个 HTTP 服务器托管 Web 前端页面（Leaflet 地图 + 画图工具）。
作为 ROS2 节点运行，支持 use_sim_time。

浏览器访问: http://localhost:8080
"""

import os
import http.server
import socketserver
import threading
from functools import partial

import rclpy
from rclpy.node import Node

class ReusableTCPServer(socketserver.TCPServer):
    """允许 launch 快速重启后立即重新绑定同一端口。"""

    allow_reuse_address = True


class WebServerNode(Node):
    """HTTP 服务器节点，托管 Web 前端页面"""

    def __init__(self):
        super().__init__("web_frontend_server")

        # 参数
        self.declare_parameter("port", 8080)
        self.declare_parameter("web_dir", "")
        self.port = self.get_parameter("port").value
        web_dir = self.get_parameter("web_dir").value

        # 如果 web_dir 为空，先尝试安装目录，再回退到源目录
        if not web_dir:
            try:
                from ament_index_python.packages import get_package_share_directory
                install_dir = os.path.join(
                    get_package_share_directory("mower_coverage"),
                    "web_frontend",
                )
                if os.path.isdir(install_dir):
                    web_dir = install_dir
            except Exception:
                pass

        # 回退到源目录（开发模式）
        if not web_dir or not os.path.isdir(web_dir):
            pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            web_dir = os.path.join(pkg_dir, "web_frontend")

        # 确保目录存在
        if not os.path.isdir(web_dir):
            os.makedirs(web_dir, exist_ok=True)
            self.get_logger().warn(f"Web 目录不存在，已创建: {web_dir}")

        # 启动 HTTP 服务器线程
        self._server = None
        self._startup_event = threading.Event()
        self._startup_error = None
        self._thread = threading.Thread(
            target=self._run_server,
            args=(web_dir,),
            daemon=True,
        )
        self._thread.start()

        # 等待后台线程完成端口绑定，避免 launch 显示“已启动”但 8080 实际没监听。
        if not self._startup_event.wait(timeout=2.0):
            self.get_logger().warn("HTTP 服务器启动超时，端口监听状态未知")
        elif self._startup_error:
            raise RuntimeError(f"HTTP 服务器启动失败: {self._startup_error}")

        self.get_logger().info(f"📁 服务目录: {web_dir}")

    def _run_server(self, web_dir):
        """运行 HTTP 服务器"""
        handler = http.server.SimpleHTTPRequestHandler

        class QuietHandler(handler):
            def log_message(self, format, *args):
                pass  # 安静模式，不打印每个请求

            # ★ 禁止缓存 — 确保前端文件修改后浏览器立即获取最新版本
            def end_headers(self):
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.send_header("Pragma", "no-cache")
                self.send_header("Expires", "0")
                super().end_headers()

        try:
            request_handler = partial(QuietHandler, directory=web_dir)
            with ReusableTCPServer(("", self.port), request_handler) as httpd:
                self._server = httpd
                self.get_logger().info(f"🌐 Web 前端服务器正在监听: http://localhost:{self.port}")
                self._startup_event.set()
                httpd.serve_forever()
        except Exception as e:
            self._startup_error = e
            self._startup_event.set()
            self.get_logger().error(f"HTTP 服务器错误: {e}")
            raise

    def shutdown(self):
        if self._server:
            self._server.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
