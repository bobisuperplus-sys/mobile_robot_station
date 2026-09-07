# -*- coding: utf-8 -*-
"""
MSH (Mobile Station Host) 后台主控微服务与通信总线模块

导出：
1. get_robot_capabilities: 全车功能能力清单生成器
2. ServiceHandler: MSH 核心业务调度中心
3. MSHServer: TCP JSON-RPC 2.0 服务网关
4. MSHClient: 客户端通信 SDK
"""

from msh.capabilities import get_robot_capabilities
from msh.client import MSHClient
from msh.rpc_server import MSHServer
from msh.service_handler import ServiceHandler

__all__ = [
    "get_robot_capabilities",
    "ServiceHandler",
    "MSHServer",
    "MSHClient",
]
