# -*- coding: utf-8 -*-
"""
MSH (Mobile Station Host) 客户端 SDK 与命令行工具 (Client SDK & CLI)

功能：
1. 提供 Python 原生 MSHClient 类，支持在上位机、脚本或外部程序中调用 RPC 接口；
2. 提供 CLI 命令行工具，支持在终端一键查询机器人全车能力清单、系统状态、遥控底盘与下发导航目标。
"""

import argparse
import json
import socket
import sys
import time
from typing import Any, Dict, Optional


class MSHClient:
    """MSH TCP JSON-RPC 2.0 客户端封装"""

    def __init__(self, host: str = "127.0.0.1", port: int = 9001, timeout: float = 5.0):
        self.host = host
        self.port = int(port)
        self.timeout = float(timeout)
        self._req_id = 0

    def call(self, method: str, params: Optional[Any] = None) -> Any:
        """发送 JSON-RPC 2.0 请求并接收返回结果"""
        self._req_id += 1
        req = {
            "jsonrpc": "2.0",
            "method": method,
            "params": params if params is not None else {},
            "id": self._req_id,
        }
        msg = json.dumps(req, ensure_ascii=False) + "\n"

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.timeout)
        try:
            sock.connect((self.host, self.port))
            sock.sendall(msg.encode("utf-8"))

            f = sock.makefile("r", encoding="utf-8")
            line = f.readline()
            if not line:
                raise RuntimeError("服务端关闭了连接，未返回响应")

            resp = json.loads(line.strip())
            if "error" in resp and resp["error"] is not None:
                err = resp["error"]
                raise RuntimeError(f"RPC Error [{err.get('code')}]: {err.get('message')}")
            return resp.get("result")
        finally:
            sock.close()

    def get_capabilities(self) -> Dict[str, Any]:
        """查询小车全套功能能力清单"""
        return self.call("system.get_capabilities")

    def get_status(self) -> Dict[str, Any]:
        """获取小车当前状态"""
        return self.call("system.get_status")

    def drive(self, linear_x: float, angular_z: float) -> Dict[str, Any]:
        """控制小车移动"""
        return self.call("teleop.drive", {"linear_x": linear_x, "angular_z": angular_z})

    def emergency_stop(self) -> Dict[str, Any]:
        """急停刹车"""
        return self.call("teleop.emergency_stop")

    def start_mapping(self) -> Dict[str, Any]:
        """启动 SLAM 建图"""
        return self.call("slam.start_mapping")

    def stop_mapping(self) -> Dict[str, Any]:
        """停止 SLAM 建图"""
        return self.call("slam.stop_mapping")

    def save_map(self, map_name: str, description: str = "") -> Dict[str, Any]:
        """保存地图"""
        return self.call("slam.save_map", {"map_name": map_name, "description": description})

    def list_maps(self) -> Dict[str, Any]:
        """列举地图库"""
        return self.call("map.list_maps")

    def load_map(self, map_name: str) -> Dict[str, Any]:
        """加载地图"""
        return self.call("map.load_map", {"map_name": map_name})

    def navigate_to(self, goal_x: float, goal_y: float) -> Dict[str, Any]:
        """下发自主导航目标点"""
        return self.call("nav.navigate_to", {"goal_x": goal_x, "goal_y": goal_y})

    def cancel_navigation(self) -> Dict[str, Any]:
        """取消导航"""
        return self.call("nav.cancel_goal")

    def get_telemetry(self) -> Dict[str, Any]:
        """获取遥测信息"""
        return self.call("telemetry.get_full_status")


def print_formatted_capabilities(cap: Dict[str, Any]) -> None:
    """格式化打印小车全套能力清单"""
    sys_info = cap.get("system", {})
    caps = cap.get("capabilities", {})

    print("======================================================================")
    print(f"  移动机器人全车功能能力清单 (MSH Capabilities Manifest)")
    print("======================================================================")
    print(f"[*] 机器人代号: {sys_info.get('robot_name')} (型号: {sys_info.get('model')}, 版本: {sys_info.get('version')})")
    print(f"[*] 通信协议:   {sys_info.get('communication_protocol')} (默认端口: {sys_info.get('default_rpc_port')})")
    print(f"[*] 支持模式:   {', '.join(sys_info.get('supported_modes', []))}")
    print(f"[*] 架构特征:   {sys_info.get('architecture')}")
    print(f"[*] 描述:       {sys_info.get('description')}")
    print("----------------------------------------------------------------------")
    print("  核心能力与支持的 RPC 服务接口矩阵:")
    print("----------------------------------------------------------------------")

    for key, item in caps.items():
        name = item.get("name", key)
        desc = item.get("description", "")
        print(f"\n[模块: {key.upper()}] - {name}")
        print(f"  说明: {desc}")

        methods = item.get("methods")
        if methods:
            print("  可用 RPC 方法:")
            for m in methods:
                params_str = ", ".join([f"{k}: {v}" for k, v in m.get("params", {}).items()]) or "无参数"
                print(f"    - {m.get('method')}({params_str})")
                print(f"      作用: {m.get('description')}")

        devices = item.get("devices")
        if devices:
            print("  配置物理设备:")
            for d in devices:
                print(f"    - {d.get('name')} ({d.get('type')}) | 采样频率: {d.get('freq_hz', '-')} Hz")

        channels = item.get("channels")
        if channels:
            print("  推流频道:")
            for c in channels:
                print(f"    - {c.get('channel')}: 端口 {c.get('port')} ({c.get('protocol')})")

    print("\n======================================================================")


def main():
    parser = argparse.ArgumentParser(description="MSH 移动机器人控制台客户端工具")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="MSH 服务端主机 IP")
    parser.add_argument("--port", type=int, default=9001, help="MSH 服务端端口")
    parser.add_argument("--capabilities", action="store_true", help="查询并打印全车功能能力清单")
    parser.add_argument("--status", action="store_true", help="获取当前系统运行状态")
    parser.add_argument("--maps", action="store_true", help="列举存储库中的所有地图")
    parser.add_argument("--drive", nargs=2, type=float, metavar=("V", "W"), help="遥控小车移动 (线速度 v, 角速度 w)")
    parser.add_argument("--stop", action="store_true", help="紧急刹车")
    parser.add_argument("--nav", nargs=2, type=float, metavar=("X", "Y"), help="下发自主导航目标点 (x, y)")
    parser.add_argument("--cancel-nav", action="store_true", help="取消自主导航任务")
    parser.add_argument("--telemetry", action="store_true", help="获取综合遥测快照")
    args = parser.parse_args()

    client = MSHClient(host=args.host, port=args.port)

    try:
        if args.capabilities:
            cap = client.get_capabilities()
            print_formatted_capabilities(cap)
        elif args.status:
            res = client.get_status()
            print(json.dumps(res, indent=2, ensure_ascii=False))
        elif args.maps:
            res = client.list_maps()
            print(json.dumps(res, indent=2, ensure_ascii=False))
        elif args.drive:
            res = client.drive(args.drive[0], args.drive[1])
            print(json.dumps(res, indent=2, ensure_ascii=False))
        elif args.stop:
            res = client.emergency_stop()
            print(json.dumps(res, indent=2, ensure_ascii=False))
        elif args.nav:
            res = client.navigate_to(args.nav[0], args.nav[1])
            print(json.dumps(res, indent=2, ensure_ascii=False))
        elif args.cancel_nav:
            res = client.cancel_navigation()
            print(json.dumps(res, indent=2, ensure_ascii=False))
        elif args.telemetry:
            res = client.get_telemetry()
            print(json.dumps(res, indent=2, ensure_ascii=False))
        else:
            # 默认打印能力清单
            cap = client.get_capabilities()
            print_formatted_capabilities(cap)
    except Exception as err:
        print(f"[MSH Client 错误] 通信失败: {err}")
        sys.exit(1)


if __name__ == "__main__":
    main()
