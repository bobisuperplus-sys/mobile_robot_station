#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
终端非阻塞键盘事件读取与遥控调度模块 (Teleoperation Controller)
提供在不挂起主仿真线程的前提下的键盘按键解析。
"""

import sys
import select
import termios
import tty


class TeleopKeyboard:
    """POSIX 终端非阻塞键盘输入控制器"""

    def __init__(self):
        self.old_settings = None
        self._is_raw = False
        if sys.stdin.isatty():
            self.old_settings = termios.tcgetattr(sys.stdin)

    def enable_raw_mode(self):
        """进入终端非阻塞无回显模式"""
        if self.old_settings is not None and not self._is_raw:
            tty.setcbreak(sys.stdin.fileno())
            self._is_raw = True

    def restore_normal_mode(self):
        """恢复终端标准模式"""
        if self.old_settings is not None and self._is_raw:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            self._is_raw = False

    def poll_key(self) -> str | None:
        """非阻塞轮询单个按键字符，若无按键则立即返回 None"""
        if not sys.stdin.isatty():
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if readable:
            return sys.stdin.read(1)
        return None

    def __enter__(self):
        self.enable_raw_mode()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.restore_normal_mode()
