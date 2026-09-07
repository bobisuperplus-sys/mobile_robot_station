#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
差速移动机器人运动学驱动模块 (Robot Kinematics Driver)
提供双轮差速正逆运动学解算、速度安全限幅与航位推算功能。
"""

import math


class TurtleBot3BurgerDriver:
    """
    TurtleBot3 Burger 双轮差速底盘驱动器
    物理参数依据 Robotis 官方工程规范:
      - 驱动轮半径 (wheel_radius): 0.033 m
      - 轮间距 (wheel_base): 0.160 m
      - 最大额定线速度: 0.22 m/s
      - 最大额定角速度: 2.84 rad/s
    """

    def __init__(self, wheel_radius: float = 0.033, wheel_base: float = 0.160):
        self.r = float(wheel_radius)
        self.L = float(wheel_base)

        self.v = 0.0
        self.w = 0.0

        self.max_v = 0.22
        self.max_w = 2.84

        self.v_step = 0.02
        self.w_step = 0.15

    def step_forward(self):
        """平滑增加前进线速度"""
        self.v = min(self.v + self.v_step, self.max_v)

    def step_backward(self):
        """平滑增加后退线速度"""
        self.v = max(self.v - self.v_step, -self.max_v)

    def step_turn_left(self):
        """平滑增加逆时针旋转角速度 (左转)"""
        self.w = min(self.w + self.w_step, self.max_w)

    def step_turn_right(self):
        """平滑增加顺时针旋转角速度 (右转)"""
        self.w = max(self.w - self.w_step, -self.max_w)

    def emergency_stop(self):
        """紧急制动刹车"""
        self.v = 0.0
        self.w = 0.0

    def get_wheel_angular_velocities(self) -> tuple[float, float]:
        """
        根据当前目标线速度与角速度，解算左右轮期望角速度 (单位: rad/s)
        差速逆运动学公式:
            omega_left  = (v - w * L / 2) / r
            omega_right = (v + w * L / 2) / r
        """
        omega_left = (self.v - self.w * self.L / 2.0) / self.r
        omega_right = (self.v + self.w * self.L / 2.0) / self.r
        return omega_left, omega_right
