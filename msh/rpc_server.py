# -*- coding: utf-8 -*-
"""
MSH (Mobile Station Host) TCP JSON-RPC 2.0 服务网关 (RPC Server)

核心规范：
1. 协议标准: 严格遵循 JSON-RPC 2.0 规范 (方法分发、参数校验与标准错误码);
2. 架构模式: 基于 Python 标准库 socketserver 实现的多线程并发网络服务端;
3. 通信格式: 行分隔 (Line-delimited JSON, \\n 结尾)，兼容 Qt、Python、Node.js 与 Linux 命令行 netcat;
4. 容错与防御: 自动捕获语法错误 (-32700)、无效请求 (-32600)、方法未找到 (-32601) 与内部错误 (-32603)。
"""

import json
import socket
import socketserver
import threading
from typing import Any, Dict, Optional

from msh.service_handler import ServiceHandler


class JSONRPCRequestHandler(socketserver.StreamRequestHandler):
    """处理单个 TCP 长连接上的行分隔 JSON-RPC 请求流"""

    def handle(self) -> None:
        handler: ServiceHandler = self.server.service_handler  # type: ignore

        for raw_line in self.rfile:
            line = raw_line.decode("utf-8").strip()
            if not line:
                continue

            req_id = None
            response: Dict[str, Any] = {"jsonrpc": "2.0"}

            try:
                # 1. 解析 JSON 报文
                try:
                    payload = json.loads(line)
                except Exception as err:
                    response["error"] = {"code": -32700, "message": f"Parse error: {err}"}
                    response["id"] = None
                    self._send_response(response)
                    continue

                if not isinstance(payload, dict):
                    response["error"] = {"code": -32600, "message": "Invalid Request: expected JSON object"}
                    response["id"] = None
                    self._send_response(response)
                    continue

                req_id = payload.get("id")
                response["id"] = req_id
                method = payload.get("method")
                params = payload.get("params")

                if not method or not isinstance(method, str):
                    response["error"] = {"code": -32600, "message": "Invalid Request: missing method name"}
                    self._send_response(response)
                    continue

                # 2. 调度业务处理函数
                try:
                    result = handler.dispatch(method, params)
                    response["result"] = result
                except KeyError as err:
                    response["error"] = {"code": -32601, "message": f"Method not found: {err}"}
                except ValueError as err:
                    response["error"] = {"code": -32602, "message": f"Invalid params: {err}"}
                except Exception as err:
                    response["error"] = {"code": -32603, "message": f"Internal error: {err}"}

            except Exception as outer_err:
                response["error"] = {"code": -32603, "message": f"Server error: {outer_err}"}
                response["id"] = req_id

            self._send_response(response)

    def _send_response(self, response: Dict[str, Any]) -> None:
        """发送 JSON 响应行"""
        try:
            resp_bytes = (json.dumps(response, ensure_ascii=False) + "\n").encode("utf-8")
            self.wfile.write(resp_bytes)
            self.wfile.flush()
        except Exception:
            pass


class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    """支持多客户端并发连接的线程化 TCP 服务器"""
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address, RequestHandlerClass, service_handler: ServiceHandler):
        super().__init__(server_address, RequestHandlerClass)
        self.service_handler = service_handler


class MSHServer:
    """MSH 主控微服务守护进程封装"""

    def __init__(
        self,
        service_handler: Optional[ServiceHandler] = None,
        host: str = "0.0.0.0",
        port: int = 9001,
    ) -> None:
        self.host = host
        self.port = int(port)
        self.service_handler = service_handler or ServiceHandler()
        self.server: Optional[ThreadedTCPServer] = None
        self._thread: Optional[threading.Thread] = None
        self._is_running = False

    def start(self, blocking: bool = False) -> None:
        """启动 MSH JSON-RPC 服务器"""
        self.server = ThreadedTCPServer((self.host, self.port), JSONRPCRequestHandler, self.service_handler)
        self._is_running = True

        if blocking:
            print(f"[MSH Server] 正在监听 TCP JSON-RPC 2.0 服务: {self.host}:{self.port} (阻塞模式)...")
            try:
                self.server.serve_forever()
            except KeyboardInterrupt:
                self.stop()
        else:
            self._thread = threading.Thread(target=self.server.serve_forever, daemon=True)
            self._thread.start()
            print(f"[MSH Server] 正在后台监听 TCP JSON-RPC 2.0 服务: {self.host}:{self.port}")

    def stop(self) -> None:
        """停止服务"""
        if self.server is not None:
            self._is_running = False
            self.server.shutdown()
            self.server.server_close()
            print(f"[MSH Server] 服务已安全关闭: {self.host}:{self.port}")

    @property
    def is_running(self) -> bool:
        return self._is_running
