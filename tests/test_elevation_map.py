# -*- coding: utf-8 -*-
"""2.5D 高程图与三维地形可通行性评估单元测试"""

import math
import unittest
import numpy as np
from navigation.elevation_map import ElevationMap2D


class TestElevationMap(unittest.TestCase):
    """测试 2.5D 高程图、坡度梯度与台阶可通行性解算"""

    def setUp(self):
        self.emap = ElevationMap2D(
            resolution=0.1,
            size_x=10.0,
            size_y=10.0,
            origin_x=-5.0,
            origin_y=-5.0,
            robot_radius=0.15,
            inflation_radius=0.35,
            max_slope_deg=15.0,
            max_step_height=0.06,
        )

    def test_flat_ground_elevation_and_zero_slope(self):
        """测试平坦地面的高程感知与零坡度评估"""
        # 生成水平地面点云 (Z = 0.0)
        xs = np.linspace(-3.0, 3.0, 50)
        ys = np.linspace(-3.0, 3.0, 50)
        xx, yy = np.meshgrid(xs, ys)
        zz = np.zeros_like(xx)
        pts = np.column_stack([xx.flatten(), yy.flatten(), zz.flatten()])

        self.emap.update_from_point_cloud(pts)

        # 检查中心点高程
        elev = self.emap.get_elevation(0.0, 0.0)
        self.assertAlmostEqual(elev, 0.0, places=2)

        # 检查中心点坡度应接近 0 度
        slope = self.emap.get_slope_deg(0.0, 0.0)
        self.assertLess(slope, 1.0)

        # 平地代价值应为 0 (完全自由空间)
        cost = self.emap.get_cost_world(0.0, 0.0)
        self.assertEqual(cost, ElevationMap2D.FREE_SPACE)

    def test_traversable_ramp_and_slope_gradient(self):
        """测试 10 度可通行缓坡的梯度解算与通过性放行"""
        # 生成沿 Y 方向倾角为 10 度的缓坡 (Z = Y * tan(10 deg))
        slope_angle = math.radians(10.0)
        xs = np.linspace(-1.0, 1.0, 30)
        ys = np.linspace(0.0, 2.0, 40)
        xx, yy = np.meshgrid(xs, ys)
        zz = yy * math.tan(slope_angle)
        pts = np.column_stack([xx.flatten(), yy.flatten(), zz.flatten()])

        self.emap.update_from_point_cloud(pts)

        # 检查坡道中部高程与坡度
        mid_y = 1.0
        expected_z = mid_y * math.tan(slope_angle)
        self.assertAlmostEqual(self.emap.get_elevation(0.0, mid_y), expected_z, delta=0.05)

        slope_detected = self.emap.get_slope_deg(0.0, mid_y)
        self.assertGreater(slope_detected, 6.0)
        self.assertLess(slope_detected, 14.0)

        # 坡道在最大限制 15 度以内，不应判定为致命障碍
        cost = self.emap.get_cost_world(0.0, mid_y)
        self.assertLess(cost, ElevationMap2D.LETHAL_OBSTACLE)
        self.assertGreater(cost, ElevationMap2D.FREE_SPACE)

    def test_vertical_wall_and_step_blocking(self):
        """测试垂直高台阶与建筑立面阻断为致命障碍 (Lethal Obstacle)"""
        # 生成一道高度 0.20m 的垂直台阶 (Y >= 1.0 时 Z = 0.20m, Y < 1.0 时 Z = 0.0m)
        xs = np.linspace(-1.0, 1.0, 20)
        ys_low = np.linspace(0.0, 0.95, 20)
        ys_high = np.linspace(1.05, 2.0, 20)

        xx_l, yy_l = np.meshgrid(xs, ys_low)
        pts_low = np.column_stack([xx_l.flatten(), yy_l.flatten(), np.zeros_like(xx_l).flatten()])

        xx_h, yy_h = np.meshgrid(xs, ys_high)
        pts_high = np.column_stack([xx_h.flatten(), yy_h.flatten(), np.full_like(xx_h, 0.20).flatten()])

        pts = np.vstack([pts_low, pts_high])
        self.emap.update_from_point_cloud(pts)

        # 检查台阶交界处的台阶高差应大于 max_step_height (0.06m) 并被标记为致命障碍
        cost_step = self.emap.get_cost_world(0.0, 1.0)
        self.assertEqual(cost_step, ElevationMap2D.LETHAL_OBSTACLE)
        self.assertTrue(self.emap.is_in_collision(0.0, 1.0))


if __name__ == '__main__':
    unittest.main()
