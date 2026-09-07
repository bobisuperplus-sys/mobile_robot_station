#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo 仿真物理世界封装模块 (World Simulation Manager)
负责场景装载、物理步进积分、底盘位姿计算、3D 激光雷达点云发生与 IMU 传感器遥测数据提取。
"""

import os
import math
import numpy as np

try:
    import mujoco
except ImportError as err:
    raise ImportError("未检测到 mujoco 库，请激活指定 Python 虚拟环境。") from err

from src.simulation.lidar_sim import RaycastLidar3D
from src.simulation.imu_sim import RealisticIMUSimulator


class UrbanWorldSimulation:
    """
    3D 建筑群仿真环境与底盘多传感器管理器
    """

    def __init__(self, scene_xml_path: str, lidar: RaycastLidar3D | None = None, imu_sim: RealisticIMUSimulator | None = None):
        if not os.path.isabs(scene_xml_path):
            scene_xml_path = os.path.abspath(scene_xml_path)

        if not os.path.exists(scene_xml_path):
            raise FileNotFoundError(f"场景配置文件不存在: {scene_xml_path}")

        self.model = mujoco.MjModel.from_xml_path(scene_xml_path)
        self.data = mujoco.MjData(self.model)

        # 缓存关键句柄
        self.robot_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.left_motor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_left_motor")
        self.right_motor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_right_motor")

        self.imu_accel_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_accel")
        self.imu_gyro_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")

        # 挂载传感器引擎 (若未提供则使用默认参数自动初始化)
        self.lidar = lidar if lidar is not None else RaycastLidar3D()
        self.imu_sim = imu_sim if imu_sim is not None else RealisticIMUSimulator()

        # 立即执行一次前向正运动学解算，初始化所有几何体与安装站点的空间坐标
        mujoco.mj_forward(self.model, self.data)

        # 离屏渲染器缓存
        self._renderer = None
        self._render_width = 640
        self._render_height = 480

    @property
    def timestep(self) -> float:
        """物理单步积分时间 (秒)"""
        return float(self.model.opt.timestep)

    def apply_wheel_controls(self, w_left: float, w_right: float):
        """向左驱动轮与右驱动轮下发角速度目标 (rad/s)"""
        self.data.ctrl[self.left_motor_id] = float(w_left)
        self.data.ctrl[self.right_motor_id] = float(w_right)

    def step(self):
        """单步物理推进"""
        mujoco.mj_step(self.model, self.data)

    def get_robot_position(self) -> np.ndarray:
        """获取小车当前世界坐标 [x, y, z] (单位: 米)"""
        return np.copy(self.data.xpos[self.robot_body_id])

    def get_robot_yaw(self) -> float:
        """从底盘位姿四元数计算航向角 Yaw (单位: 弧度)"""
        quat = self.data.xquat[self.robot_body_id]
        w, x, y, z = quat[0], quat[1], quat[2], quat[3]
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_raw_imu_telemetry(self) -> dict[str, np.ndarray]:
        """获取物理引擎原生未滤波的理想 6 轴 IMU 加速度 (m/s^2) 与角速度 (rad/s)"""
        accel = np.copy(self.data.sensor("imu_accel").data)
        gyro = np.copy(self.data.sensor("imu_gyro").data)
        return {
            "acceleration": accel,
            "angular_velocity": gyro,
        }

    def get_realistic_imu_telemetry(self) -> dict[str, np.ndarray]:
        """获取注入白噪声与随机游走偏置的高保真 IMU 遥测数据"""
        raw = self.get_raw_imu_telemetry()
        return self.imu_sim.update(
            accel_true=raw["acceleration"],
            gyro_true=raw["angular_velocity"],
            dt=self.timestep,
        )

    def get_lidar_pointcloud(self, return_world_frame: bool = False) -> dict[str, np.ndarray]:
        """
        触发单次 3D 激光雷达光线投射扫描
        :param return_world_frame: 若为 True 返回全局世界坐标点云，若为 False 返回小车局部系点云
        :return: 包含 points (K, 3), intensities (K,), rings (K,) 等数据的字典
        """
        return self.lidar.scan(
            model=self.model,
            data=self.data,
            lidar_site_name="lidar_site",
            robot_body_name="base_link",
            return_world_frame=return_world_frame,
        )

    def render_camera(self, camera_name: str = "front_cam", width: int = 640, height: int = 480) -> np.ndarray:
        """
        离屏渲染指定相机的 RGB 图像帧
        :param camera_name: 相机名称 (如 'front_cam' 或 'overview_cam')
        :param width: 画面宽度 (默认 640)
        :param height: 画面高度 (默认 480)
        :return: (height, width, 3) 的 uint8 RGB 数组
        """
        if self._renderer is None or self._render_width != width or self._render_height != height:
            self._render_width = width
            self._render_height = height
            self._renderer = mujoco.Renderer(self.model, height=height, width=width)

        self._renderer.update_scene(self.data, camera=camera_name)
        return self._renderer.render()
