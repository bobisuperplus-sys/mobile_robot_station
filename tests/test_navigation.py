# -*- coding: utf-8 -*-
"""自主导航算法单元测试用例

测试内容：
1. Costmap2D 高程切片、栅格量化与欧氏距离场双层安全膨胀；
2. A* 全局路径规划器在迷宫障碍物下的避障搜索与视线剪枝平滑；
3. DWA 动态窗口局部规划器动态速度窗口采样与避障收敛性验证。
"""

import os
import sys
import unittest
import numpy as np

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from navigation.costmap import Costmap2D
from navigation.global_planner import GlobalPlannerAStar
from navigation.local_planner import LocalPlannerDWA


class TestNavigation(unittest.TestCase):
    """测试 2D 代价地图与自主导航规划器"""

    def test_costmap_slice_and_inflation(self):
        """测试 3D 点云切片与欧氏距离安全膨胀层"""
        costmap = Costmap2D(
            resolution=0.05,
            size_x=4.0,
            size_y=4.0,
            origin_x=-2.0,
            origin_y=-2.0,
            robot_radius=0.10,
            inflation_radius=0.20,
            z_min=0.06,
            z_max=0.40,
        )

        # 构造包含地面点 (Z=0)、立面障碍点 (Z=0.2) 和高空点 (Z=1.5) 的测试点云
        pts = np.array(
            [
                [0.0, 0.0, 0.0],    # 地面点 (应被切片过滤)
                [0.0, 0.0, 0.2],    # 立面障碍点 (应被保留)
                [0.0, 0.0, 1.5],    # 高空点 (应被切片过滤)
            ],
            dtype=np.float64,
        )

        costmap.update_from_point_cloud(pts)

        # 障碍物中心栅格应为致命障碍 (254)
        u_center, v_center = costmap.world_to_map(0.0, 0.0)
        self.assertEqual(costmap.get_cost(u_center, v_center), Costmap2D.LETHAL_OBSTACLE)

        # 在物理距离 0.08m 处 (<= robot_radius 0.10m)，应为内切圆碰撞区 (253)
        u_inscribed, v_inscribed = costmap.world_to_map(0.08, 0.0)
        self.assertEqual(costmap.get_cost(u_inscribed, v_inscribed), Costmap2D.INSCRIBED_INFLATED_OBSTACLE)

        # 在物理距离 0.20m 处 (处于膨胀衰减区)，代价应介于 1 ~ 252 之间
        u_inflated, v_inflated = costmap.world_to_map(0.20, 0.0)
        cost_inflated = costmap.get_cost(u_inflated, v_inflated)
        self.assertTrue(Costmap2D.INFLATED_MIN_COST <= cost_inflated <= 252)

        # 在物理距离 0.50m 处 (超出 robot_radius + inflation_radius 0.30m)，应为完全自由区 (0)
        u_free, v_free = costmap.world_to_map(0.50, 0.0)
        self.assertEqual(costmap.get_cost(u_free, v_free), Costmap2D.FREE_SPACE)

    def test_astar_global_planner_wall_avoidance(self):
        """测试 A* 全局规划器在横向障碍墙场景下的避障绕行与剪枝平滑"""
        costmap = Costmap2D(
            resolution=0.05,
            size_x=6.0,
            size_y=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
            robot_radius=0.10,
            inflation_radius=0.15,
        )

        # 在 X=0 处设置一道横向障碍墙 (Y 从 -2.5 到 1.0)，在 Y=1.0 到 2.5 留出通行缺口
        costmap.set_obstacle_rect(x_min=-0.1, x_max=0.1, y_min=-2.5, y_max=1.0)

        planner = GlobalPlannerAStar(costmap, cost_weight=2.0)

        # 起点在墙左侧 (-1.5, 0.0)，终点在墙右侧 (1.5, 0.0)
        start = (-1.5, 0.0)
        goal = (1.5, 0.0)

        path = planner.plan(start, goal, smooth=True)
        self.assertIsNotNone(path, "A* 规划器应成功找到绕行路径")
        self.assertGreater(len(path), 1)

        # 验证起点与终点正确
        self.assertAlmostEqual(path[0][0], start[0], places=2)
        self.assertAlmostEqual(path[0][1], start[1], places=2)
        self.assertAlmostEqual(path[-1][0], goal[0], places=2)
        self.assertAlmostEqual(path[-1][1], goal[1], places=2)

        # 验证路径上的所有点均不与致命障碍物重叠
        for pt in path:
            self.assertFalse(costmap.is_lethal_world(pt[0], pt[1]), f"路径点 {pt} 不得位于致命障碍区")

    def test_dwa_local_planner_commands(self):
        """测试 DWA 局部规划器速度输出与避障决策"""
        costmap = Costmap2D(
            resolution=0.05,
            size_x=6.0,
            size_y=6.0,
            origin_x=-3.0,
            origin_y=-3.0,
            robot_radius=0.11,
            inflation_radius=0.20,
        )

        # 在小车前方 (X=0.8, Y=0.0) 放置立柱障碍
        costmap.set_obstacle_rect(x_min=0.7, x_max=0.9, y_min=-0.2, y_max=0.2)

        dwa = LocalPlannerDWA(costmap, max_vel_x=0.5, max_vel_theta=1.2)

        # 小车在原点 (0, 0)，朝向正东 (theta=0)，全局路径引导绕行航点位于障碍侧方 (1.5, 0.6)
        current_pose = (0.0, 0.0, 0.0)
        current_vel = (0.2, 0.0)
        subgoal = (1.5, 0.6)

        vx, vw, traj = dwa.compute_velocity_commands(current_pose, current_vel, subgoal, dt=0.1)

        # 面对绕行航点，DWA 应输出正向角速度驱动小车转向避障
        self.assertGreater(vw, 0.0, "DWA 面对侧方绕行航点应输出向左转角速度")
        self.assertGreater(vx, 0.0, "DWA 应保持前向安全行进速度")
        self.assertGreater(len(traj), 0)

        # 验证推演轨迹末端不发生致命碰撞
        end_pt = traj[-1]
        self.assertFalse(costmap.is_lethal_world(end_pt[0], end_pt[1]))

        # 测试终点精准停靠：距离目标只有 0.05m 时，应主动刹车 (vx = 0, vw = 0)
        near_pose = (1.48, 0.60, 0.0)
        stop_vx, stop_vw, _ = dwa.compute_velocity_commands(
            near_pose, (0.1, 0.0), subgoal, dt=0.1, is_final_goal=True
        )
        self.assertEqual(stop_vx, 0.0)
        self.assertEqual(stop_vw, 0.0)


if __name__ == "__main__":
    unittest.main()
