#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
多传感器模块单元与性能基准测试用例
"""

import sys
import os
import time
import unittest
import numpy as np

# 导入工程根路径
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from src.simulation.world_sim import UrbanWorldSimulation
from src.simulation.lidar_sim import RaycastLidar3D
from src.simulation.imu_sim import RealisticIMUSimulator


class TestSensors(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene_path = os.path.join(PROJECT_ROOT, "assets", "scenes", "urban_world.xml")
        cls.sim = UrbanWorldSimulation(cls.scene_path)

    def test_lidar_pointcloud_generation(self):
        """测试 3D 激光雷达点云发生器输出格式与有效性"""
        scan_local = self.sim.get_lidar_pointcloud(return_world_frame=False)
        self.assertIn("points", scan_local)
        self.assertIn("intensities", scan_local)
        self.assertIn("rings", scan_local)

        points = scan_local["points"]
        self.assertGreater(len(points), 1000, "有效点云数量过少，光线求交可能存在穿透或漏报")
        self.assertEqual(points.shape[1], 3, "点云坐标必须为 (N, 3)")
        self.assertEqual(points.dtype, np.float32)

        # 测距范围检查
        distances = np.linalg.norm(points, axis=1)
        self.assertTrue(np.all(distances >= 0.10), "存在小于最小测距盲区的异常点")
        self.assertTrue(np.all(distances <= 25.5), "存在超出最大测距量程的异常点")

        # 检查世界坐标点云
        scan_world = self.sim.get_lidar_pointcloud(return_world_frame=True)
        world_pts = scan_world["points"]
        # 地面高度应该在 0 附近
        self.assertTrue(np.any(np.abs(world_pts[:, 2]) < 0.1), "点云中未检测到测试场地面的反射点")

    def test_lidar_benchmark_speed(self):
        """测试 3D 激光雷达单帧求交耗时是否满足实时性能要求"""
        lidar = RaycastLidar3D(vertical_channels=16, horizontal_resolution_deg=3.0)
        times = []
        for _ in range(10):
            t0 = time.perf_counter()
            _ = lidar.scan(self.sim.model, self.sim.data)
            dt = (time.perf_counter() - t0) * 1000.0
            times.append(dt)

        avg_time = np.mean(times)
        print(f"\n[性能测试] 16 线 3D 激光雷达平均生成耗时: {avg_time:.2f} ms")
        self.assertLess(avg_time, 80.0, "单帧 Raycasting 耗时超过实时阈值 (80ms)")

    def test_realistic_imu_noise_and_drift(self):
        """测试 6 轴 IMU 噪声与漂移特性"""
        imu = RealisticIMUSimulator(
            accel_noise_std=0.05,
            accel_bias_walk_std=0.001,
            gyro_noise_std=0.01,
            gyro_bias_walk_std=0.0005,
        )

        accel_true = np.array([0.0, 0.0, 9.81])
        gyro_true = np.array([0.0, 0.0, 0.0])

        accel_samples = []
        for _ in range(200):
            out = imu.update(accel_true, gyro_true, dt=0.005)
            accel_samples.append(out["accel_meas"])

        samples = np.array(accel_samples)
        # 统计平均值与真值的偏差 (由于偏置的存在会有偏移)
        mean_accel = np.mean(samples, axis=0)
        self.assertAlmostEqual(mean_accel[2], 9.81, delta=0.2)
        std_accel = np.std(samples, axis=0)
        self.assertAlmostEqual(std_accel[0], 0.05, delta=0.02)


if __name__ == "__main__":
    unittest.main()
