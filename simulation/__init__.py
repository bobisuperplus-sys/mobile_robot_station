# -*- coding: utf-8 -*-
"""
移动机器人仿真、动力学、传感器与多媒体推流核心模块
"""

# 首先导入平台兼容层，确保 GLFW 与 Qt 优先选用 X11/XWayland 后端
import simulation.platform_compat

from simulation.robot_driver import TurtleBot3BurgerDriver
from simulation.world_sim import UrbanWorldSimulation
from simulation.teleop import TeleopKeyboard
from simulation.lidar_sim import RaycastLidar3D
from simulation.imu_sim import RealisticIMUSimulator
from simulation.streamer import GStreamerStreamer, MultiCameraStreamServer

__all__ = [
    "TurtleBot3BurgerDriver",
    "UrbanWorldSimulation",
    "TeleopKeyboard",
    "RaycastLidar3D",
    "RealisticIMUSimulator",
    "GStreamerStreamer",
    "MultiCameraStreamServer",
]
