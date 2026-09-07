# -*- coding: utf-8 -*-
"""
3D LIO-SLAM 激光惯导里程计与空间建图单元测试用例
"""

import os
import sys
import unittest
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from core_math.se3 import SE3
from slam.preprocess import PointcloudPreprocessor
from slam.imu_tracker import IMUKinematicTracker
from slam.lio_odometry import LIOOdometry


class TestSLAM(unittest.TestCase):
    """测试 SLAM 预处理、IMU 追踪与点到面 ICP 建图"""

    def test_preprocessor_range_and_voxel(self):
        """测试测距过滤与体素降采样"""
        pre = PointcloudPreprocessor(voxel_size=0.1, min_range=0.5, max_range=10.0)

        # 包含自遮挡点 (<0.5m)、有效点 (2~5m) 与超远点 (>10m)
        pts = np.array(
            [
                [0.1, 0.0, 0.0],    # 距离 0.1 (剔除)
                [1.0, 0.0, 0.0],    # 距离 1.0 (保留)
                [1.02, 0.01, 0.01], # 同一小体素内的临近点
                [15.0, 0.0, 0.0],   # 距离 15.0 (剔除)
            ],
            dtype=np.float64,
        )

        filtered = pre.range_filter(pts)
        self.assertEqual(len(filtered), 2)

        downsampled = pre.voxel_downsample(filtered)
        self.assertEqual(len(downsampled), 1)
        # 质心应在 [1.01, 0.005, 0.005] 附近
        self.assertAlmostEqual(downsampled[0, 0], 1.01, places=3)

    def test_imu_kinematic_tracker_damping(self):
        """测试 IMU 运动学积分与非全息侧向/垂直速度抑制"""
        tracker = IMUKinematicTracker(gravity=9.81)

        # 首次初始化
        tracker.propagate(gyro=[0, 0, 0], accel=[0, 0, 9.81], dt=0.01)

        # 模拟产生侧向加速度扰动
        for _ in range(10):
            tracker.propagate(gyro=[0, 0, 0], accel=[0.5, 0.8, 9.81], dt=0.01)

        # 检查速度: 侧向速度 (Y) 与垂直速度 (Z) 应受到强烈阻尼抑制
        vel = tracker.velocity
        self.assertLess(abs(vel[1]), 0.2)
        self.assertLess(abs(vel[2]), 0.2)

    def test_lio_odometry_point_to_plane_registration(self):
        """测试构建合成平面环境下的点到面 ICP 配准收敛性"""
        odom = LIOOdometry(voxel_size=0.05, map_voxel_size=0.05, max_iterations=10)

        # 构造一个合成的 L 型走廊点云 (地面 + 前方墙面 + 侧面墙面)
        ground_x, ground_y = np.meshgrid(np.linspace(-2, 2, 20), np.linspace(-2, 2, 20))
        ground = np.column_stack([ground_x.ravel(), ground_y.ravel(), np.zeros(400)])

        wall_y, wall_z = np.meshgrid(np.linspace(-2, 2, 20), np.linspace(0, 2, 20))
        wall_front = np.column_stack([np.full(400, 2.0), wall_y.ravel(), wall_z.ravel()])

        wall_x, wall_z2 = np.meshgrid(np.linspace(-2, 2, 20), np.linspace(0, 2, 20))
        wall_side = np.column_stack([wall_x.ravel(), np.full(400, 2.0), wall_z2.ravel()])

        room_pts = np.vstack([ground, wall_front, wall_side])

        # 1. 注册首帧 (原点)
        pose1, res1 = odom.register_frame(room_pts)
        self.assertAlmostEqual(pose1.translation[0], 0.0, places=3)
        self.assertGreater(len(odom.get_map_points()), 100)

        # 2. 模拟机器人向前平移 0.15 米，向右偏转 0.05 弧度
        # 则新观测到的局部点云相当于被反向变换: P_local = T_inv * P_world
        true_motion = SE3.from_xyz_rpy(0.15, 0.02, 0.0, 0.0, 0.0, 0.05)
        new_room_pts = true_motion.inverse().act(room_pts)

        # 注入先验预测并进行配准
        odom.predict_with_imu(gyro=[0, 0, 0.05 / 0.1], accel=[0.15 / 0.01, 0, 9.81], dt=0.1)
        pose2, res2 = odom.register_frame(new_room_pts)

        # 验证估算位姿与真实运动接近
        est_x, est_y, est_z, _, _, est_yaw = pose2.to_xyz_rpy()
        self.assertAlmostEqual(est_x, 0.15, delta=0.03)
        self.assertAlmostEqual(est_yaw, 0.05, delta=0.02)
        self.assertLess(res2, 0.05)


if __name__ == "__main__":
    unittest.main()
