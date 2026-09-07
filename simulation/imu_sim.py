#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
6 轴高频 IMU 真实物理误差仿真模块 (Realistic IMU Simulator)
模拟高精工业 MEMS 惯导的高斯白噪声与时间累积随机游走偏置 (Bias Instability & Random Walk)。
"""

import numpy as np


class RealisticIMUSimulator:
    """
    6 轴高频 IMU 传感器误差模型
    测量方程:
      tilde_accel = accel_true + bias_accel + white_noise_accel
      tilde_gyro  = gyro_true  + bias_gyro  + white_noise_gyro

      bias(t + dt) = bias(t) + sqrt(dt) * bias_walk_std * N(0, I)
    """

    def __init__(
        self,
        accel_noise_std: float = 0.02,
        accel_bias_walk_std: float = 0.0001,
        gyro_noise_std: float = 0.005,
        gyro_bias_walk_std: float = 0.00005,
        init_accel_bias: list[float] | None = None,
        init_gyro_bias: list[float] | None = None,
    ):
        self.accel_noise_std = float(accel_noise_std)
        self.accel_bias_walk_std = float(accel_bias_walk_std)
        self.gyro_noise_std = float(gyro_noise_std)
        self.gyro_bias_walk_std = float(gyro_bias_walk_std)

        # 内部状态：当前偏置 (Bias)
        if init_accel_bias is not None:
            self.accel_bias = np.array(init_accel_bias, dtype=np.float64)
        else:
            self.accel_bias = np.zeros(3, dtype=np.float64)

        if init_gyro_bias is not None:
            self.gyro_bias = np.array(init_gyro_bias, dtype=np.float64)
        else:
            self.gyro_bias = np.zeros(3, dtype=np.float64)

    def update(
        self,
        accel_true: np.ndarray,
        gyro_true: np.ndarray,
        dt: float = 0.005,
    ) -> dict[str, np.ndarray]:
        """
        输入 MuJoCo 真实物理真值，注入高斯白噪声与随机游走偏置
        :param accel_true: (3,) 真实加速度 [ax, ay, az] (m/s^2)
        :param gyro_true: (3,) 真实角速度 [wx, wy, wz] (rad/s)
        :param dt: 采样时间间隔 (秒)
        :return: 包含测量值与真值的字典
        """
        accel_true = np.asarray(accel_true, dtype=np.float64)
        gyro_true = np.asarray(gyro_true, dtype=np.float64)
        sqrt_dt = math_sqrt = np.sqrt(max(dt, 1e-6))

        # 1. 随机游走偏置演化 (Brownian Motion)
        if self.accel_bias_walk_std > 0.0:
            self.accel_bias += np.random.normal(0.0, self.accel_bias_walk_std * sqrt_dt, size=3)
        if self.gyro_bias_walk_std > 0.0:
            self.gyro_bias += np.random.normal(0.0, self.gyro_bias_walk_std * sqrt_dt, size=3)

        # 2. 高斯白噪声 (White Noise)
        accel_noise = np.random.normal(0.0, self.accel_noise_std, size=3) if self.accel_noise_std > 0.0 else np.zeros(3)
        gyro_noise = np.random.normal(0.0, self.gyro_noise_std, size=3) if self.gyro_noise_std > 0.0 else np.zeros(3)

        # 3. 测量输出
        accel_meas = accel_true + self.accel_bias + accel_noise
        gyro_meas = gyro_true + self.gyro_bias + gyro_noise

        return {
            "accel_meas": accel_meas.astype(np.float32),
            "gyro_meas": gyro_meas.astype(np.float32),
            "accel_true": accel_true.astype(np.float32),
            "gyro_true": gyro_true.astype(np.float32),
            "accel_bias": np.copy(self.accel_bias).astype(np.float32),
            "gyro_bias": np.copy(self.gyro_bias).astype(np.float32),
        }

    def reset(self):
        """重置偏置"""
        self.accel_bias.fill(0.0)
        self.gyro_bias.fill(0.0)
