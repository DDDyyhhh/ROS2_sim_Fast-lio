#!/usr/bin/env python3
"""
test_web_server.py — 回归测试：Web 前端 HTTP 服务器应能可靠重新绑定端口。

ROS2 launch 重启时，旧 web_server 刚退出后端口可能短时间处于 TIME_WAIT；
如果 TCPServer 没有启用 SO_REUSEADDR，会导致 [Errno 98] Address already in use，
进程存在但 8080 没有监听。
"""

import contextlib
import http.server
import os
import socket
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from mower_coverage.web_server import ReusableTCPServer


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format, *args):
        pass


def _free_port():
    with contextlib.closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def test_http_server_allows_immediate_port_reuse():
    """web_server 关闭后应能立刻重新绑定同一端口，避免 launch 重启时报 Errno 98。"""
    port = _free_port()

    first = ReusableTCPServer(('127.0.0.1', port), QuietHandler)
    first.server_close()

    second = ReusableTCPServer(('127.0.0.1', port), QuietHandler)
    second.server_close()


if __name__ == "__main__":
    test_http_server_allows_immediate_port_reuse()
    print("✅ web_server 回归测试通过")
