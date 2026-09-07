# -*- coding: utf-8 -*-
"""
6 轴 IMU 运动学追踪与差速底盘非全息约束预测器
在激光雷达扫描间歇进行高频惯性积分，提供紧耦合位姿先验，并主动抑制平坦路面 Z 轴退化
"""

import numpy as np
from core_math.so3 import SO3
from core_math.se3 import SE3


class IMUKinematicTracker:
    """
    IMU 运动学积分与运动先验生成器
    """

    def __init__(
        self,
        gravity: float = 9.81,
        damping_lateral: float = 0.95,
        damping_vertical: float = 0.95,
    ):
        """
        参数:
            gravity: 重力加速度大小 (m/s^2)
            damping_lateral: 横向滑移阻尼衰减系数 (0~1, 贴合差速底盘非全息约束)
            damping_vertical: 垂直跳跃阻尼衰减系数 (0~1, 抑制 Z 轴积分发散)
        """
        self.g_vec = np.array([0.0, 0.0, -gravity], dtype=np.float64)
        self.damping_lateral = float(damping_lateral)
        self.damping_vertical = float(damping_vertical)

        # 状态量: 世界系位姿、世界系速度、零偏估计
        self.pose = SE3.identity()
        self.velocity = np.zeros(3, dtype=np.float64)
        self.gyro_bias = np.zeros(3, dtype=np.float64)
        self.accel_bias = np.zeros(3, dtype=np.float64)

        self._last_gyro = np.zeros(3, dtype=np.float64)
        self._last_accel = np.zeros(3, dtype=np.float64)
        self._initialized = False

    def reset(self, initial_pose: SE3 = None, initial_velocity: np.ndarray = None):
        """重置积分器状态"""
        self.pose = SE3.identity() if initial_pose is None else initial_pose
        self.velocity = np.zeros(3, dtype=np.float64) if initial_velocity is None else np.asarray(initial_velocity, dtype=np.float64)
        self.gyro_bias = np.zeros(3, dtype=np.float64)
        self.accel_bias = np.zeros(3, dtype=np.float64)
        self._initialized = False

    def propagate(self, gyro: np.ndarray, accel: np.ndarray, dt: float) -> SE3:
        """
        执行单步 IMU 运动学积分 (采用中点积分法与非全息软约束)

        参数:
            gyro: 当前角速度测量值 (rad/s, 车体局部坐标系)
            accel: 当前包含重力的线加速度测量值 (m/s^2, 车体局部坐标系)
            dt: 积分时间步长 (秒)
        返回:
            更新后的先验位姿 SE3
        """
        gyro_curr = np.asarray(gyro, dtype=np.float64).reshape(3) - self.gyro_bias
        accel_curr = np.asarray(accel, dtype=np.float64).reshape(3) - self.accel_bias

        if not self._initialized:
            self._last_gyro = gyro_curr
            self._last_accel = accel_curr
            self._initialized = True
            return self.pose

        # 1. 旋转积分 (采用中点平均角速度)
        gyro_mid = 0.5 * (self._last_gyro + gyro_curr)
        delta_rot = SO3.exp(gyro_mid * dt)
        r_old = self.pose.rotation
        r_new = r_old @ delta_rot

        # 2. 线加速度转换至世界系并消除重力影响
        accel_mid = 0.5 * (self._last_accel + accel_curr)
        accel_world = r_old.act(accel_mid) + self.g_vec

        # 3. 平移与速度更新
        new_pos = self.pose.translation + self.velocity * dt + 0.5 * accel_world * (dt**2)
        new_vel = self.velocity + accel_world * dt

        # 4. 轮式差速底盘非全息动力学约束注入 (Non-holonomic Constraints)
        # 将速度投影回当前车体局部坐标系
        r_inv = r_new.inverse()
        vel_body = r_inv.act(new_vel)

        # 差速轮式机器人在正常行驶时，侧向速度 (Y) 与垂直速度 (Z) 理论上极小
        # 对侧向与垂直分量实施阻尼抑制，杜绝 Z 轴与横向漂移
        vel_body[1] *= (1.0 - self.damping_lateral * min(1.0, dt * 10.0))
        vel_body[2] *= (1.0 - self.damping_vertical * min(1.0, dt * 10.0))

        # 重新映射回世界系
        self.velocity = r_new.act(vel_body)
        self.pose = SE3(r_new, new_pos)

        # 缓存上一时刻测量
        self._last_gyro = gyro_curr
        self._last_accel = accel_curr

        return self.pose

    def set_pose(self, corrected_pose: SE3):
        """用激光雷达配准后的高精度位姿校准当前先验"""
        self.pose = corrected_pose
