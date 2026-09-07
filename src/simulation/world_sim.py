#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MuJoCo 仿真物理世界封装模块 (World Simulation Manager)
负责场景装载、物理步进积分、底盘位姿计算与 IMU 传感器遥测数据提取。
"""

import os
import math
import numpy as np

try:
    import mujoco
except ImportError as err:
    raise ImportError("未检测到 mujoco 库，请激活指定 Python 虚拟环境。") from err


class UrbanWorldSimulation:
    """
    3D 建筑群仿真环境与底盘传感器管理器
    """

    def __init__(self, scene_xml_path: str):
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
        # base_link 的全局四元数在 xquat
        quat = self.data.xquat[self.robot_body_id]
        w, x, y, z = quat[0], quat[1], quat[2], quat[3]
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def get_imu_telemetry(self) -> dict[str, np.ndarray]:
        """获取当前 6 轴高频 IMU 真实加速度 (m/s^2) 与角速度 (rad/s)"""
        accel = np.copy(self.data.sensor("imu_accel").data)
        gyro = np.copy(self.data.sensor("imu_gyro").data)
        return {
            "acceleration": accel,
            "angular_velocity": gyro,
        }
