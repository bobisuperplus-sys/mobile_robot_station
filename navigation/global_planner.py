# -*- coding: utf-8 -*-
"""A* 全局路径规划器与视线剪枝平滑模块

功能：
1. 8 邻域代价感知 A* 启发式图搜索 (Cost-Aware A*)；
2. 严密的斜向穿墙 (Corner Cutting) 物理防护；
3. 将安全膨胀层代价值融入搜索权重，驱使小车优先走开阔路中；
4. 视线交叉检测 (Line-of-Sight Bresenham) 路径平滑剪枝。
"""

import heapq
import math
from typing import List, Optional, Tuple
import numpy as np

from navigation.costmap import Costmap2D


class GlobalPlannerAStar:
    """基于 A* 算法的代价感知全局路径规划器"""

    # 8 邻域位移及几何距离 (du, dv, base_cost)
    SQRT2 = math.sqrt(2.0)
    MOTIONS = [
        (1, 0, 1.0),
        (-1, 0, 1.0),
        (0, 1, 1.0),
        (0, -1, 1.0),
        (1, 1, SQRT2),
        (1, -1, SQRT2),
        (-1, 1, SQRT2),
        (-1, -1, SQRT2),
    ]

    def __init__(self, costmap: Costmap2D, cost_weight: float = 3.0) -> None:
        """初始化全局规划器

        参数:
            costmap: 2D 代价栅格地图实例
            cost_weight: 代价感知权重 (越大越远离膨胀层走马路中心)
        """
        self.costmap = costmap
        self.cost_weight = float(cost_weight)

    def plan(
        self,
        start_world: Tuple[float, float],
        goal_world: Tuple[float, float],
        smooth: bool = True,
    ) -> Optional[List[Tuple[float, float]]]:
        """执行从起点到终点的全局路径规划

        参数:
            start_world: 起点世界物理坐标 (x, y)
            goal_world: 终点世界物理坐标 (x, y)
            smooth: 是否执行视线剪枝平滑

        返回:
            规划成功返回世界物理坐标航点列表 [(x0, y0), (x1, y1), ...]，规划失败返回 None
        """
        su, sv = self.costmap.world_to_map(start_world[0], start_world[1])
        gu, gv = self.costmap.world_to_map(goal_world[0], goal_world[1])

        # 边界有效性检查
        if not self.costmap.is_in_bounds(su, sv):
            return None
        if not self.costmap.is_in_bounds(gu, gv):
            return None

        # 目标点若是致命障碍，在周围搜寻最近最低代价的可通行邻近点
        if self.costmap.is_lethal(gu, gv):
            gu, gv = self._find_nearest_free(gu, gv, max_radius=24)
            if gu is None or gv is None:
                return None

        # 起点若位于碰撞膨胀区或紧贴障碍，安全逃逸至最近开阔通行点
        if self.costmap.is_lethal(su, sv):
            su, sv = self._find_nearest_free(su, sv, max_radius=24)
            if su is None or sv is None:
                return None

        if su == gu and sv == gv:
            return [start_world, goal_world]

        # 执行 8 邻域 A* 搜索
        raw_grid_path = self._search(su, sv, gu, gv)
        if not raw_grid_path:
            return None

        # 视线剪枝平滑
        if smooth and len(raw_grid_path) > 2:
            grid_path = self._smooth_path(raw_grid_path)
        else:
            grid_path = raw_grid_path

        # 转换为物理世界坐标
        world_path = [self.costmap.map_to_world(u, v) for u, v in grid_path]

        # 精确将起点和终点替换为输入的世界物理坐标
        world_path[0] = start_world
        world_path[-1] = goal_world

        # 对航线进行等间距密集插值，为局部规划器提供连续切线引导
        return self._interpolate_path(world_path, step_size=0.15)

    def _interpolate_path(
        self, path: List[Tuple[float, float]], step_size: float = 0.15
    ) -> List[Tuple[float, float]]:
        """对稀疏航路点进行等间距密集插值"""
        if len(path) <= 1:
            return path
        dense_path = [path[0]]
        for i in range(len(path) - 1):
            p1 = np.array(path[i], dtype=np.float64)
            p2 = np.array(path[i + 1], dtype=np.float64)
            segment_len = float(np.linalg.norm(p2 - p1))
            num_segments = max(1, int(np.ceil(segment_len / step_size)))
            for s in range(1, num_segments + 1):
                pt = p1 + (p2 - p1) * (float(s) / num_segments)
                dense_path.append((float(pt[0]), float(pt[1])))
        return dense_path

    def _search(self, su: int, sv: int, gu: int, gv: int) -> Optional[List[Tuple[int, int]]]:
        """底层 A* 栅格搜索核心"""
        nx, ny = self.costmap.nx, self.costmap.ny
        res = self.costmap.resolution
        cost_arr = self.costmap.cost_array

        g_score = np.full((ny, nx), np.inf, dtype=np.float32)
        closed = np.zeros((ny, nx), dtype=bool)
        parent_u = np.full((ny, nx), -1, dtype=np.int32)
        parent_v = np.full((ny, nx), -1, dtype=np.int32)

        g_score[sv, su] = 0.0
        h_start = math.hypot(gu - su, gv - sv) * res

        # 优先队列元素: (f_score, counter, u, v)
        counter = 0
        open_set: List[Tuple[float, int, int, int]] = []
        heapq.heappush(open_set, (h_start, counter, su, sv))

        while open_set:
            f, _, curr_u, curr_v = heapq.heappop(open_set)

            if closed[curr_v, curr_u]:
                continue
            closed[curr_v, curr_u] = True

            # 抵达目标点
            if curr_u == gu and curr_v == gv:
                return self._reconstruct_path(parent_u, parent_v, su, sv, gu, gv)

            curr_g = g_score[curr_v, curr_u]

            for du, dv, step_dist in self.MOTIONS:
                nu = curr_u + du
                nv = curr_v + dv

                if not (0 <= nu < nx and 0 <= nv < ny):
                    continue
                if closed[nv, nu]:
                    continue

                cell_cost = int(cost_arr[nv, nu])
                # 致命障碍与内切圆碰撞区严禁通行
                if cell_cost >= Costmap2D.INSCRIBED_INFLATED_OBSTACLE:
                    continue

                # 斜穿墙角保护 (Corner Cutting): 斜向移动时，相邻两个正交格不得均为障碍
                if du != 0 and dv != 0:
                    cost_adj1 = int(cost_arr[curr_v, curr_u + du])
                    cost_adj2 = int(cost_arr[curr_v + dv, curr_u])
                    if (cost_adj1 >= Costmap2D.INSCRIBED_INFLATED_OBSTACLE and
                            cost_adj2 >= Costmap2D.INSCRIBED_INFLATED_OBSTACLE):
                        continue

                # 代价感知计算：结合几何距离与膨胀层惩罚
                # cell_cost: 0 (自由区) ~ 252 (接近致命区)
                cost_penalty = 1.0 + self.cost_weight * (float(cell_cost) / 252.0)
                tentative_g = curr_g + (step_dist * res) * cost_penalty

                if tentative_g < g_score[nv, nu]:
                    g_score[nv, nu] = tentative_g
                    parent_u[nv, nu] = curr_u
                    parent_v[nv, nu] = curr_v

                    # 启发式欧氏距离
                    h = math.hypot(gu - nu, gv - nv) * res
                    f_score = tentative_g + h

                    counter += 1
                    heapq.heappush(open_set, (f_score, counter, nu, nv))

        return None

    def _reconstruct_path(
        self,
        parent_u: np.ndarray,
        parent_v: np.ndarray,
        su: int,
        sv: int,
        gu: int,
        gv: int,
    ) -> List[Tuple[int, int]]:
        """从回溯矩阵构建栅格路径"""
        path = [(gu, gv)]
        curr_u, curr_v = gu, gv

        while not (curr_u == su and curr_v == sv):
            pu = int(parent_u[curr_v, curr_u])
            pv = int(parent_v[curr_v, curr_u])
            if pu == -1 or pv == -1:
                break
            curr_u, curr_v = pu, pv
            path.append((curr_u, curr_v))

        path.reverse()
        return path

    def _find_nearest_free(
        self, u: int, v: int, max_radius: int = 24
    ) -> Tuple[Optional[int], Optional[int]]:
        """在以 (u, v) 为中心的方框螺旋环中搜索最近且代价值最低的非致命通行点"""
        for r in range(1, max_radius + 1):
            candidates = []
            for du in range(-r, r + 1):
                for dv in (-r, r):
                    nu, nv = u + du, v + dv
                    if self.costmap.is_in_bounds(nu, nv) and not self.costmap.is_lethal(nu, nv):
                        candidates.append((self.costmap.get_cost(nu, nv), nu, nv))
            for dv in range(-r + 1, r):
                for du in (-r, r):
                    nu, nv = u + du, v + dv
                    if self.costmap.is_in_bounds(nu, nv) and not self.costmap.is_lethal(nu, nv):
                        candidates.append((self.costmap.get_cost(nu, nv), nu, nv))
            if candidates:
                # 优先选择当前层代价最低（最开阔）的安全航点
                candidates.sort(key=lambda item: item[0])
                return candidates[0][1], candidates[0][2]
        return None, None

    def _smooth_path(self, path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """基于 Bresenham 视线交叉碰撞检测的贪心路径平滑剪枝"""
        if len(path) <= 2:
            return path

        smoothed: List[Tuple[int, int]] = [path[0]]
        curr_idx = 0

        while curr_idx < len(path) - 1:
            # 从末端向后贪心搜寻最远可见航点
            furthest_idx = curr_idx + 1
            for check_idx in range(len(path) - 1, curr_idx, -1):
                if self._line_of_sight(path[curr_idx], path[check_idx]):
                    furthest_idx = check_idx
                    break

            smoothed.append(path[furthest_idx])
            curr_idx = furthest_idx

        return smoothed

    def _line_of_sight(self, p1: Tuple[int, int], p2: Tuple[int, int], max_cost: int = 100) -> bool:
        """Bresenham 光线投射，检测两栅格点连线上是否存在高代价或致命障碍"""
        u0, v0 = p1
        u1, v1 = p2

        du = abs(u1 - u0)
        dv = abs(v1 - v0)
        su = 1 if u0 < u1 else -1
        sv = 1 if v0 < v1 else -1
        err = du - dv

        curr_u, curr_v = u0, v0

        while True:
            # 检测当前栅格是否不可通行或切入高阻滞膨胀区
            if not self.costmap.is_in_bounds(curr_u, curr_v):
                return False
            if self.costmap.get_cost(curr_u, curr_v) >= max_cost:
                return False

            if curr_u == u1 and curr_v == v1:
                break

            e2 = 2 * err
            if e2 > -dv:
                err -= dv
                curr_u += su
            if e2 < du:
                err += du
                curr_v += sv

        return True
