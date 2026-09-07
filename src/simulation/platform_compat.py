#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Linux 桌面显示后端平台兼容模块 (Platform Compatibility)
针对 Ubuntu 22.04 / 24.04 等默认 Wayland 会话环境，强制启用 X11 (XWayland) 兼容层，
消除 GLFW 与 Qt 视口因 Wayland 窗口坐标协议限制而触发的警告与兼容性异常。
"""

import os
import warnings


def init_platform_compatibility():
    """初始化 Linux 显示后端环境变量与警告过滤器"""
    # 强制 GLFW 视口库采用 X11 (XWayland) 协议后端
    if "GLFW_PLATFORM" not in os.environ:
        os.environ["GLFW_PLATFORM"] = "x11"

    # 强制 Qt 平台抽象插件 (QPA) 优先使用 xcb (X11) 后端
    if "QT_QPA_PLATFORM" not in os.environ:
        os.environ["QT_QPA_PLATFORM"] = "xcb"

    # 过滤 GLFW 在 Wayland 平台下无法获取窗口绝对坐标的无害 UserWarning 提示
    warnings.filterwarnings("ignore", message=".*Wayland: The platform does not provide the window position.*")


# 模块导入时自动无缝执行环境注入
init_platform_compatibility()
