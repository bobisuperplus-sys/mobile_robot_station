# -*- coding: utf-8 -*-
"""
2.5D 高程图 (Elevation Map) 与三维地形可通行性评估模块

核心功能：
1. 空间 3D 激光点云二维柱状离散化与地面高程提取 (Ground Elevation Estimation)；
2. 基于二维空间梯度的地形坡度角计算 (Slope Gradient, theta = arctan(|grad(z)|))；
3. 基于邻域极差的局部台阶高度解算 (Local Step Height, Delta h)；
4. 综合坡度与台阶的多准则可通行性评估 (Traversability Evaluation)：
   - 平坦路面：低代价通行 (Cost ~ 0)；
   - 缓坡平台/坡道 (Slope <= max_slope): 连续线性代价值放行 (Cost 1 ~ 120)，引导小车爬坡；
   - 陡坡 (Slope > max_slope) 或 垂直硬台阶 (Step > max_step): 判定为不可逾越致命障碍 (Cost = 254)；
5. 兼容 Costmap2D 标准接口，无缝无感接入 A* 全局路径规划与 DWA 局部动态避障。
"""

import math
from typing import Optional, Tuple
import numpy as np
from scipy.ndimage import distance_transform_edt, maximum_filter, minimum_filter


class ElevationMap2D:
    """2.5D 地形高程与可通行性代价栅格地图"""

    FREE_SPACE: int = 0
    INFLATED_MIN_COST: int = 1
    RAMP_MIN_COST: int = 15
    INSCRIBED_INFLATED_OBSTACLE: int = 253
    LETHAL_OBSTACLE: int = 254
    NO_INFORMATION: int = 255

    def __init__(
        self,
        resolution: float = 0.05,
        size_x: float = 24.0,
        size_y: float = 24.0,
        origin_x: float = -12.0,
        origin_y: float = -12.0,
        robot_radius: float = 0.12,
        inflation_radius: float = 0.35,
        decay_factor: float = 6.0,
        max_slope_deg: float = 16.0,
        max_step_height: float = 0.06,
        default_ground_z: float = 0.0,
    ) -> None:
        """初始化 2.5D 高程与可通行性地图"""
        self.resolution = float(resolution)
        self.size_x = float(size_x)
        self.size_y = float(size_y)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)

        self.robot_radius = float(robot_radius)
        self.inflation_radius = float(inflation_radius)
        self.decay_factor = float(decay_factor)
        self.max_slope_deg = float(max_slope_deg)
        self.max_step_height = float(max_step_height)
        self.default_ground_z = float(default_ground_z)

        self.nx = int(np.ceil(self.size_x / self.resolution))
        self.ny = int(np.ceil(self.size_y / self.resolution))

        # 核心层数据 (行优先 ny 对应 Y，列 nx 对应 X)
        self.elevation_array = np.full((self.ny, self.nx), self.default_ground_z, dtype=np.float32)
        self.slope_deg_array = np.zeros((self.ny, self.nx), dtype=np.float32)
        self.step_height_array = np.zeros((self.ny, self.nx), dtype=np.float32)
        self.cost_array = np.zeros((self.ny, self.nx), dtype=np.uint8)
        self.lethal_mask = np.zeros((self.ny, self.nx), dtype=bool)
        self.valid_mask = np.zeros((self.ny, self.nx), dtype=bool)

    def reset(self) -> None:
        """重置全部高程与代价数据"""
        self.elevation_array.fill(self.default_ground_z)
        self.slope_deg_array.fill(0.0)
        self.step_height_array.fill(0.0)
        self.cost_array.fill(self.FREE_SPACE)
        self.lethal_mask.fill(False)
        self.valid_mask.fill(False)

    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """物理世界坐标转栅格索引 (u, v)"""
        u = int(np.floor((x - self.origin_x) / self.resolution))
        v = int(np.floor((y - self.origin_y) / self.resolution))
        return u, v

    def map_to_world(self, u: int, v: int) -> Tuple[float, float]:
        """栅格索引 (u, v) 转物理世界坐标"""
        x = self.origin_x + (u + 0.5) * self.resolution
        y = self.origin_y + (v + 0.5) * self.resolution
        return x, y

    def is_valid(self, u: int, v: int) -> bool:
        """检查栅格索引是否在地图物理范围内"""
        return 0 <= u < self.nx and 0 <= v < self.ny

    def is_in_bounds(self, u: int, v: int) -> bool:
        """检查栅格索引是否在地图物理范围内 (is_valid 别名)"""
        return 0 <= u < self.nx and 0 <= v < self.ny

    def is_lethal(self, u: int, v: int) -> bool:
        """检查栅格索引是否为不可通过的致命障碍"""
        if not self.is_valid(u, v):
            return True
        return bool(self.cost_array[v, u] >= self.INSCRIBED_INFLATED_OBSTACLE)

    def get_elevation(self, x: float, y: float) -> float:
        """查询物理坐标处地面高程"""
        u, v = self.world_to_map(x, y)
        if self.is_valid(u, v):
            return float(self.elevation_array[v, u])
        return self.default_ground_z

    def get_slope_deg(self, x: float, y: float) -> float:
        """查询物理坐标处坡度角度"""
        u, v = self.world_to_map(x, y)
        if self.is_valid(u, v):
            return float(self.slope_deg_array[v, u])
        return 0.0

    def get_cost(self, u: int, v: int) -> int:
        """查询指定栅格索引 (u, v) 处的代价值 (对齐 Costmap2D 标准接口)"""
        if not self.is_valid(u, v):
            return self.NO_INFORMATION
        return int(self.cost_array[v, u])

    def get_cost_grid(self, u: int, v: int) -> int:
        """查询指定栅格索引处的代价值 (get_cost 别名)"""
        return self.get_cost(u, v)

    def get_cost_world(self, x: float, y: float) -> int:
        """查询世界物理坐标 (x, y) 处的代价值"""
        u, v = self.world_to_map(x, y)
        return self.get_cost(u, v)

    def is_in_collision(self, x: float, y: float, threshold: int = 253) -> bool:
        """判断世界物理坐标处是否发生碰撞"""
        return self.get_cost_world(x, y) >= threshold

    def update_from_point_cloud(self, points: np.ndarray) -> None:
        """接收空间 3D 点云并解算 2.5D 高程、坡度、台阶及可通行性膨胀代价

        参数:
            points: 空间三维点云坐标，形状为 (N, 3)
        """
        self.reset()
        if len(points) == 0:
            return

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        # 空间边界过滤
        in_bounds = (
            (x >= self.origin_x)
            & (x < self.origin_x + self.size_x)
            & (y >= self.origin_y)
            & (y < self.origin_y + self.size_y)
        )
        if not np.any(in_bounds):
            return

        valid_x = x[in_bounds]
        valid_y = y[in_bounds]
        valid_z = z[in_bounds]

        u_arr = np.floor((valid_x - self.origin_x) / self.resolution).astype(int)
        v_arr = np.floor((valid_y - self.origin_y) / self.resolution).astype(int)
        u_arr = np.clip(u_arr, 0, self.nx - 1)
        v_arr = np.clip(v_arr, 0, self.ny - 1)

        # 线性展平索引
        flat_idx = v_arr * self.nx + u_arr
        total_cells = self.ny * self.nx

        # 1. 统计各柱状单元的高程范围与点分布
        # 为各单元计算 min_z 和 max_z
        # 使用快速向量化排序分组
        order = np.argsort(flat_idx)
        sorted_flat = flat_idx[order]
        sorted_z = valid_z[order]

        unique_cells, start_indices = np.unique(sorted_flat, return_index=True)
        end_indices = np.append(start_indices[1:], len(sorted_flat))

        tall_obstacle_flat = np.zeros(total_cells, dtype=bool)
        ground_z_flat = np.full(total_cells, self.default_ground_z, dtype=np.float32)
        valid_cells_flat = np.zeros(total_cells, dtype=bool)

        for cell_idx, s_idx, e_idx in zip(unique_cells, start_indices, end_indices):
            cell_z = sorted_z[s_idx:e_idx]
            z_min_c = np.min(cell_z)
            z_max_c = np.max(cell_z)
            valid_cells_flat[cell_idx] = True

            # 区分高耸立面垂直障碍 (建筑外墙、高立柱、高圆柱桶)
            # 若垂直高跨超过 0.35m 且顶部高于 0.40m，定为建筑墙体/实体障碍
            if (z_max_c - z_min_c > 0.35 and z_max_c > 0.40) or z_max_c > 0.60:
                tall_obstacle_flat[cell_idx] = True

            # 地面与低矮地形提取 (只取 z <= 0.45m 的点中最高的作为表面高程)
            ground_candidates = cell_z[cell_z <= 0.45]
            if len(ground_candidates) > 0:
                ground_z_flat[cell_idx] = np.max(ground_candidates)
            else:
                ground_z_flat[cell_idx] = z_max_c

        # 恢复为 (ny, nx) 二维矩阵
        self.elevation_array = ground_z_flat.reshape((self.ny, self.nx))
        self.valid_mask = valid_cells_flat.reshape((self.ny, self.nx))
        tall_obstacle_mask = tall_obstacle_flat.reshape((self.ny, self.nx))

        # 对未直接采样覆盖的网格单元，利用欧氏距离变换最近邻插值补全连续地形，杜绝假阶跃
        if np.any(self.valid_mask) and not np.all(self.valid_mask):
            _, indices = distance_transform_edt(~self.valid_mask, return_indices=True)
            self.elevation_array = self.elevation_array[indices[0], indices[1]]

        # 2. 地形坡度与台阶高度解算 (Slope Gradient & Step Height)
        # 采用轻量均值滤波抑制激光离散量化噪声
        from scipy.ndimage import uniform_filter
        filtered_elev = uniform_filter(self.elevation_array, size=3)

        # 二维梯度解算坡度
        grad_y, grad_x = np.gradient(filtered_elev, self.resolution)
        slope_rad = np.arctan(np.sqrt(grad_x**2 + grad_y**2))
        self.slope_deg_array = np.degrees(slope_rad).astype(np.float32)

        # 局部 3x3 邻域高差解算台阶高度
        local_max = maximum_filter(filtered_elev, size=3)
        local_min = minimum_filter(filtered_elev, size=3)
        self.step_height_array = (local_max - local_min).astype(np.float32)

        # 3. 判定不可通行致命障碍区 (Lethal Obstacles)
        # 垂直立面墙体、超出攀爬阈值的陡坡、或超出底盘离地间隙的高台阶
        self.lethal_mask = (
            tall_obstacle_mask
            | (self.slope_deg_array > self.max_slope_deg)
            | (self.step_height_array > self.max_step_height)
        )

        # 5. 可通行缓坡/平地赋分 (Traversable Terrain Cost)
        # 平地代价为 0，缓坡随角度线性渐增 (1 ~ 120)，引导全局规划优先走平地，但必要时可顺畅走坡道
        slope_ratio = np.clip(self.slope_deg_array / self.max_slope_deg, 0.0, 1.0)
        traversable_cost = (slope_ratio * 90.0).astype(np.uint8)
        self.cost_array = np.where(self.lethal_mask, self.LETHAL_OBSTACLE, traversable_cost)

        # 6. 基于欧氏距离变换 (EDT) 实施机器人足印内切圆与安全膨胀
        if np.any(self.lethal_mask):
            dist_transform = distance_transform_edt(~self.lethal_mask)
            dist_meters = dist_transform * self.resolution

            # 内切圆半径内：硬碰撞致命区
            inscribed_mask = dist_meters <= self.robot_radius
            self.cost_array[inscribed_mask] = self.LETHAL_OBSTACLE

            # 安全膨胀层内：指数衰减缓冲区
            inflation_mask = (dist_meters > self.robot_radius) & (dist_meters <= self.inflation_radius)
            if np.any(inflation_mask):
                norm_dist = (dist_meters[inflation_mask] - self.robot_radius) / (self.inflation_radius - self.robot_radius)
                decay_cost = (
                    self.INSCRIBED_INFLATED_OBSTACLE * np.exp(-self.decay_factor * norm_dist)
                ).astype(np.uint8)
                # 叠加膨胀代价，保留原有坡度基础代价中较大者
                self.cost_array[inflation_mask] = np.maximum(
                    self.cost_array[inflation_mask], decay_cost
                )
