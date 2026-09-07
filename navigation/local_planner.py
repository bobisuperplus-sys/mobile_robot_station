# -*- coding: utf-8 -*-
"""DWA 动态窗口局部避障与轨迹跟踪规划器

功能：
1. 动力学窗口 (Dynamic Window) 动态极限计算；
2. 差速底盘正向运动学候选轨迹离散推演 (Trajectory Rollout)；
3. 航向对准、防碰撞安全净距与行进速度多目标加权打分；
4. 终点厘米级精准减速、自旋对齐与平稳驻车逻辑。
"""

import math
from typing import List, Optional, Tuple
import numpy as np

from navigation.costmap import Costmap2D


class LocalPlannerDWA:
    """基于动态窗口法 (Dynamic Window Approach) 的局部避障规划器"""

    def __init__(
        self,
        costmap: Costmap2D,
        max_vel_x: float = 0.5,
        min_vel_x: float = 0.0,
        max_vel_theta: float = 1.2,
        acc_lim_x: float = 0.8,
        acc_lim_theta: float = 2.5,
        sim_time: float = 2.2,
        sim_granularity: float = 0.1,
        vx_samples: int = 10,
        vtheta_samples: int = 25,
        acc_window_dt: float = 0.35,
        heading_cost_weight: float = 0.60,
        obstacle_cost_weight: float = 0.15,
        velocity_cost_weight: float = 0.25,
        goal_tolerance: float = 0.15,
    ) -> None:
        """初始化 DWA 局部规划器

        参数:
            costmap: 2D 代价栅格地图实例
            max_vel_x: 最大正向前向线速度 (米/秒)
            min_vel_x: 最小前向线速度 (米/秒)
            max_vel_theta: 最大转向角速度 (弧度/秒)
            acc_lim_x: 最大线加速度 (米/秒^2)
            acc_lim_theta: 最大角加速度 (弧度/秒^2)
            sim_time: 前瞻推演预测时域 (秒)
            sim_granularity: 轨迹推演离散时间步长 (秒)
            vx_samples: 线速度离散采样分度数
            vtheta_samples: 角速度离散采样分度数
            acc_window_dt: 动态窗口采样响应时间窗口 (秒)
            heading_cost_weight: 目标航向对准评价权重
            obstacle_cost_weight: 障碍物避让安全距离评价权重
            velocity_cost_weight: 前向通行效率评价权重
            goal_tolerance: 终点停车容差半径 (米)
        """
        self.costmap = costmap
        self.max_vel_x = float(max_vel_x)
        self.min_vel_x = float(min_vel_x)
        self.max_vel_theta = float(max_vel_theta)
        self.acc_lim_x = float(acc_lim_x)
        self.acc_lim_theta = float(acc_lim_theta)

        self.sim_time = float(sim_time)
        self.sim_granularity = float(sim_granularity)
        self.vx_samples = int(vx_samples)
        self.vtheta_samples = int(vtheta_samples)
        self.acc_window_dt = float(acc_window_dt)

        self.heading_weight = float(heading_cost_weight)
        self.obstacle_weight = float(obstacle_cost_weight)
        self.velocity_weight = float(velocity_cost_weight)
        self.goal_tolerance = float(goal_tolerance)

    def compute_velocity_commands(
        self,
        current_pose: Tuple[float, float, float],
        current_vel: Tuple[float, float],
        subgoal: Tuple[float, float],
        dt: float = 0.1,
        is_final_goal: bool = False,
    ) -> Tuple[float, float, List[Tuple[float, float]]]:
        """计算当前周期的最优底盘驱动速度指令与推演轨迹

        参数:
            current_pose: 当前小车位姿 (x, y, theta)，其中 theta 为朝向角 (弧度)
            current_vel: 当前小车实际速度 (v_x, v_theta)
            subgoal: 局部路标导航点 (x_goal, y_goal)
            dt: 控制周期 (秒)
            is_final_goal: 当前 subgoal 是否为最终全局目的地

        返回:
            (best_vx, best_vtheta, best_trajectory): 最优驱动指令与轨迹
        """
        curr_x, curr_y, curr_theta = current_pose
        curr_vx, curr_vtheta = current_vel

        # 检查是否已进入最终终点停靠容差区
        dist_to_subgoal = math.hypot(subgoal[0] - curr_x, subgoal[1] - curr_y)
        if is_final_goal and dist_to_subgoal <= self.goal_tolerance:
            # 抵达最终目标，平稳刹车
            return 0.0, 0.0, [(curr_x, curr_y)]

        # 1. 计算动态速度窗口 [v_min, v_max] 与 [w_min, w_max]
        dt_acc = self.acc_window_dt if self.acc_window_dt > 0 else dt
        # 巡航途中保持最小前向速度偏置，防止在局部膨胀梯度平坦区出现零速躺平停滞
        floor_v = 0.0 if is_final_goal else max(self.min_vel_x, 0.08)
        v_min = max(floor_v, curr_vx - self.acc_lim_x * dt_acc)
        v_max = min(self.max_vel_x, curr_vx + self.acc_lim_x * dt_acc)

        w_min = max(-self.max_vel_theta, curr_vtheta - self.acc_lim_theta * dt_acc)
        w_max = min(self.max_vel_theta, curr_vtheta + self.acc_lim_theta * dt_acc)

        # 终点连续平顺减速控制：逼近最终目标时逐渐收紧最大速度，防止高速超调
        if is_final_goal:
            decel_v = max(0.06, min(self.max_vel_x, dist_to_subgoal * 0.70))
            v_max = min(v_max, decel_v)
            v_min = min(v_min, v_max)

        # 速度空间采样
        v_candidates = np.linspace(v_min, v_max, self.vx_samples)
        w_candidates = np.linspace(w_min, w_max, self.vtheta_samples)

        # 仅当航向严重偏离航标方向 (偏差大于 95 度) 时，才执行原地自旋对准，避免中途频繁切断前向速度
        angle_to_goal = math.atan2(subgoal[1] - curr_y, subgoal[0] - curr_x)
        angle_diff = self._normalize_angle(angle_to_goal - curr_theta)
        if abs(angle_diff) > math.radians(95.0):
            spin_w = math.copysign(min(abs(angle_diff) * 1.5, self.max_vel_theta), angle_diff)
            return 0.0, spin_w, [(curr_x, curr_y)]

        best_score = -float("inf")
        best_vx = 0.0
        best_vtheta = 0.0
        best_trajectory: List[Tuple[float, float]] = []

        # 收集所有可行轨迹以进行归一化打分
        valid_trajectories = []

        for v in v_candidates:
            for w in w_candidates:
                traj, is_safe, min_dist = self._rollout_trajectory(
                    curr_x, curr_y, curr_theta, v, w
                )
                if not is_safe:
                    continue

                # 评估指标计算
                # 1. 轨迹末端朝向与局部目标方位角偏差
                end_x, end_y, end_theta = traj[-1]
                target_heading = math.atan2(subgoal[1] - end_y, subgoal[0] - end_x)
                heading_err = abs(self._normalize_angle(target_heading - end_theta))
                heading_score = math.pi - heading_err

                # 2. 障碍物净距离得分 (min_dist 越大越安全)
                obstacle_score = min_dist

                # 3. 速度得分 (常规巡航鼓励快速通行，终点进站阶段鼓励平稳减速)
                if is_final_goal:
                    desired_v = max(0.05, min(0.20, dist_to_subgoal * 0.45))
                    velocity_score = -abs(v - desired_v)
                else:
                    velocity_score = v

                # 4. 航标距离推进得分 (越靠近子目标得分越高，驱使小车积极前行)
                dist_to_goal = math.hypot(subgoal[0] - end_x, subgoal[1] - end_y)
                progress_score = -dist_to_goal

                valid_trajectories.append({
                    "vx": v,
                    "vw": w,
                    "traj": [(p[0], p[1]) for p in traj],
                    "heading_score": heading_score,
                    "obstacle_score": obstacle_score,
                    "velocity_score": velocity_score,
                    "progress_score": progress_score,
                })

        if not valid_trajectories:
            # 所有前向轨迹均有碰撞风险，紧急制动并缓慢自旋探索生机
            return 0.0, 0.4 if curr_vtheta >= 0 else -0.4, [(curr_x, curr_y)]

        # 归一化各评价因子
        max_head = max(t["heading_score"] for t in valid_trajectories) + 1e-5
        max_obs = max(t["obstacle_score"] for t in valid_trajectories) + 1e-5
        max_vel = self.max_vel_x if self.max_vel_x > 0 else 1.0
        min_prog = min(t["progress_score"] for t in valid_trajectories)
        max_prog = max(t["progress_score"] for t in valid_trajectories)
        prog_range = (max_prog - min_prog) if (max_prog - min_prog) > 1e-5 else 1.0

        for item in valid_trajectories:
            norm_head = item["heading_score"] / max_head
            norm_obs = item["obstacle_score"] / max_obs
            norm_vel = item["velocity_score"] / max_vel
            norm_prog = (item["progress_score"] - min_prog) / prog_range

            total_score = (
                self.heading_weight * norm_head
                + self.obstacle_weight * norm_obs
                + self.velocity_weight * norm_vel
                + 0.50 * norm_prog
            )

            if total_score > best_score:
                best_score = total_score
                best_vx = item["vx"]
                best_vtheta = item["vw"]
                best_trajectory = item["traj"]

        return best_vx, best_vtheta, best_trajectory

    def _rollout_trajectory(
        self,
        x0: float,
        y0: float,
        theta0: float,
        vx: float,
        vw: float,
    ) -> Tuple[List[Tuple[float, float, float]], bool, float]:
        """按差速运动学模型正向推演预测时域轨迹并检测安全裕度

        返回:
            (trajectory, is_safe, min_clearance_meters)
        """
        steps = int(np.ceil(self.sim_time / self.sim_granularity))
        traj: List[Tuple[float, float, float]] = []

        x, y, theta = x0, y0, theta0
        min_clearance = float("inf")

        for _ in range(steps):
            x += vx * math.cos(theta) * self.sim_granularity
            y += vx * math.sin(theta) * self.sim_granularity
            theta += vw * self.sim_granularity
            traj.append((x, y, theta))

            # 碰撞与安全检测
            u, v = self.costmap.world_to_map(x, y)
            if not self.costmap.is_in_bounds(u, v):
                return traj, False, 0.0

            cost = self.costmap.get_cost(u, v)
            if cost >= Costmap2D.INSCRIBED_INFLATED_OBSTACLE:
                # 触及致命或小车几何外廓不可通行区
                return traj, False, 0.0

            # 估算到最近致命障碍的等效物理净空距离
            if cost == Costmap2D.FREE_SPACE:
                clearance = self.costmap.robot_radius + self.costmap.inflation_radius
            else:
                # 逆推衰减距离
                decay_ratio = 1.0 - (float(cost) / 253.0)
                clearance = self.costmap.robot_radius + decay_ratio * self.costmap.inflation_radius

            if clearance < min_clearance:
                min_clearance = clearance

        return traj, True, min_clearance

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """将角度规范化至 [-pi, pi] 区间"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_subgoal_from_path(
        self,
        current_pose: Tuple[float, float, float],
        path: List[Tuple[float, float]],
        lookahead_dist: float = 0.6,
    ) -> Tuple[float, float]:
        """从全局路径中提取符合前瞻距离的局部跟踪航点

        参数:
            current_pose: 小车当前位姿 (x, y, theta)
            path: 全局规划世界坐标航点序列 [(x, y), ...]
            lookahead_dist: 前瞻物理距离 (米)

        返回:
            (subgoal_x, subgoal_y): 局部目标点
        """
        if not path:
            return current_pose[0], current_pose[1]

        curr_x, curr_y, _ = current_pose

        # 1. 寻找距离当前车身最近的路径段索引
        min_idx = 0
        min_d = float("inf")
        for i, pt in enumerate(path):
            d = math.hypot(pt[0] - curr_x, pt[1] - curr_y)
            if d < min_d:
                min_d = d
                min_idx = i

        # 2. 从最近点向后搜寻第一个距离大于等于 lookahead_dist 的点
        for i in range(min_idx, len(path)):
            d = math.hypot(path[i][0] - curr_x, path[i][1] - curr_y)
            if d >= lookahead_dist:
                return path[i]

        return path[-1]

