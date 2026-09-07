# -*- coding: utf-8 -*-
"""
移动机器人仿真、动力学与多传感器核心模块
"""

# 首先导入平台兼容层，确保 GLFW 与 Qt 优先选用 X11/XWayland 后端
import src.simulation.platform_compat

from src.simulation.robot_driver import TurtleBot3BurgerDriver
from src.simulation.world_sim import UrbanWorldSimulation
from src.simulation.teleop import TeleopKeyboard
from src.simulation.lidar_sim import RaycastLidar3D
from src.simulation.imu_sim import RealisticIMUSimulator

__all__ = [
    "TurtleBot3BurgerDriver",
    "UrbanWorldSimulation",
    "TeleopKeyboard",
    "RaycastLidar3D",
    "RealisticIMUSimulator",
]
