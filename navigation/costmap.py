# -*- coding: utf-8 -*-
"""2D 代价栅格切片与安全膨胀地图模块

功能：
1. 将 3D 点云按移动机器人底盘净空高度窗口切片（0.06m <= Z <= 0.40m）；
2. 离散栅格化至二维空间矩阵（分辨率通常 0.05m）；
3. 基于 SciPy 欧氏距离变换 (EDT) 实现内切圆致命区与指数衰减安全膨胀层；
4. 提供世界物理坐标与栅格索引的高性能双向转换与查询。
"""

from typing import Optional, Tuple
import numpy as np
from scipy.ndimage import distance_transform_edt


class Costmap2D:
    """二维代价栅格地图类"""

    # 代价值常数定义 (符合移动机器人标准代价体系)
    FREE_SPACE: int = 0
    INFLATED_MIN_COST: int = 1
    INSCRIBED_INFLATED_OBSTACLE: int = 253
    LETHAL_OBSTACLE: int = 254
    NO_INFORMATION: int = 255

    def __init__(
        self,
        resolution: float = 0.05,
        size_x: float = 16.0,
        size_y: float = 16.0,
        origin_x: float = -8.0,
        origin_y: float = -8.0,
        robot_radius: float = 0.11,
        inflation_radius: float = 0.25,
        decay_factor: float = 8.0,
        z_min: float = 0.06,
        z_max: float = 0.40,
    ) -> None:
        """初始化 2D 代价地图

        参数:
            resolution: 栅格分辨率 (米/栅格)
            size_x: 地图 X 轴物理跨度 (米)
            size_y: 地图 Y 轴物理跨度 (米)
            origin_x: 地图左下角原点 X 物理坐标 (米)
            origin_y: 地图左下角原点 Y 物理坐标 (米)
            robot_radius: 机器人物理半径/内切圆半径 (米)
            inflation_radius: 安全膨胀层厚度 (米)
            decay_factor: 膨胀代价指数衰减系数
            z_min: 点云垂直切片下限 (米)
            z_max: 点云垂直切片上限 (米)
        """
        self.resolution = float(resolution)
        self.size_x = float(size_x)
        self.size_y = float(size_y)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)

        self.robot_radius = float(robot_radius)
        self.inflation_radius = float(inflation_radius)
        self.decay_factor = float(decay_factor)
        self.z_min = float(z_min)
        self.z_max = float(z_max)

        # 栅格尺寸 (列数 nx 对应 X 轴，行数 ny 对应 Y 轴)
        self.nx = int(np.ceil(self.size_x / self.resolution))
        self.ny = int(np.ceil(self.size_y / self.resolution))

        # 栅格矩阵 (ny, nx)，类型 uint8，默认全自由空间
        self.cost_array = np.zeros((self.ny, self.nx), dtype=np.uint8)
        # 记录原始致命障碍栅格掩码
        self.lethal_mask = np.zeros((self.ny, self.nx), dtype=bool)

    def reset(self) -> None:
        """清空代价地图"""
        self.cost_array.fill(self.FREE_SPACE)
        self.lethal_mask.fill(False)

    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """将物理世界坐标转换为栅格索引 (u, v)

        参数:
            x: 世界 X 坐标 (米)
            y: 世界 Y 坐标 (米)

        返回:
            (u, v): u 为列索引 (对应 X)，v 为行索引 (对应 Y)
        """
        u = int(np.floor((x - self.origin_x) / self.resolution))
        v = int(np.floor((y - self.origin_y) / self.resolution))
        return u, v

    def map_to_world(self, u: int, v: int) -> Tuple[float, float]:
        """将栅格索引 (u, v) 转换为对应栅格中心的世界物理坐标 (x, y)

        参数:
            u: 栅格列索引
            v: 栅格行索引

        返回:
            (x, y): 世界物理坐标 (米)
        """
        x = self.origin_x + (float(u) + 0.5) * self.resolution
        y = self.origin_y + (float(v) + 0.5) * self.resolution
        return x, y

    def is_in_bounds(self, u: int, v: int) -> bool:
        """检查栅格索引是否在地图像素边界内"""
        return 0 <= u < self.nx and 0 <= v < self.ny

    def is_in_bounds_world(self, x: float, y: float) -> bool:
        """检查世界物理坐标是否在地图物理边界内"""
        return (
            self.origin_x <= x < self.origin_x + self.size_x
            and self.origin_y <= y < self.origin_y + self.size_y
        )

    def update_from_point_cloud(self, points: np.ndarray) -> None:
        """根据 3D 点云执行高程切片与欧氏距离安全膨胀更新

        参数:
            points: (N, 3) 空间点云数组，包含 [x, y, z]
        """
        self.reset()
        if points is None or len(points) == 0:
            return

        # 1. 高程切片筛选处于小车立面高度窗口的点
        z = points[:, 2]
        slice_mask = (z >= self.z_min) & (z <= self.z_max)
        sliced_points = points[slice_mask]

        if len(sliced_points) == 0:
            return

        # 2. 计算栅格索引 (向量化投影)
        px = sliced_points[:, 0]
        py = sliced_points[:, 1]
        u_indices = np.floor((px - self.origin_x) / self.resolution).astype(np.int32)
        v_indices = np.floor((py - self.origin_y) / self.resolution).astype(np.int32)

        # 筛选有效范围内的索引
        valid = (
            (u_indices >= 0)
            & (u_indices < self.nx)
            & (v_indices >= 0)
            & (v_indices < self.ny)
        )
        u_valid = u_indices[valid]
        v_valid = v_indices[valid]

        if len(u_valid) == 0:
            return

        # 3. 标记致命障碍物栅格
        self.lethal_mask[v_valid, u_valid] = True

        # 4. 欧氏距离变换与安全膨胀 (Euclidean Distance Transform)
        self._compute_inflation()

    def set_obstacle_rect(self, x_min: float, x_max: float, y_min: float, y_max: float) -> None:
        """在地图上设置矩形实体障碍（用于几何与测试场景仿真）"""
        u_min, v_min = self.world_to_map(x_min, y_min)
        u_max, v_max = self.world_to_map(x_max, y_max)

        u_start = max(0, min(u_min, u_max))
        u_end = min(self.nx, max(u_min, u_max) + 1)
        v_start = max(0, min(v_min, v_max))
        v_end = min(self.ny, max(v_min, v_max) + 1)

        self.lethal_mask[v_start:v_end, u_start:u_end] = True
        self._compute_inflation()

    def _compute_inflation(self) -> None:
        """基于欧氏距离场执行双层安全膨胀计算"""
        if not np.any(self.lethal_mask):
            self.cost_array.fill(self.FREE_SPACE)
            return

        # 计算每个像元到最近障碍物边缘的物理欧氏距离 (单位: 米)
        # distance_transform_edt 计算输入掩码中 0 像元到最近非 0 像元的欧氏距离
        dist_grid = distance_transform_edt(~self.lethal_mask) * self.resolution

        # 1. 致命障碍物像元直接标记 LETHAL_OBSTACLE (254)
        self.cost_array[self.lethal_mask] = self.LETHAL_OBSTACLE

        # 2. 内切圆半径内像元标记 INSCRIBED_INFLATED_OBSTACLE (253)
        inscribed_mask = (~self.lethal_mask) & (dist_grid <= self.robot_radius)
        self.cost_array[inscribed_mask] = self.INSCRIBED_INFLATED_OBSTACLE

        # 3. 膨胀层区间衰减 (robot_radius < dist <= robot_radius + inflation_radius)
        inflation_zone = (
            (~self.lethal_mask)
            & (dist_grid > self.robot_radius)
            & (dist_grid <= self.robot_radius + self.inflation_radius)
        )

        if np.any(inflation_zone):
            d = dist_grid[inflation_zone] - self.robot_radius
            # 指数衰减代价计算
            decay_cost = 252.0 * np.exp(-self.decay_factor * (d / self.inflation_radius))
            decay_cost = np.clip(decay_cost, self.INFLATED_MIN_COST, 252.0).astype(np.uint8)
            self.cost_array[inflation_zone] = decay_cost

        # 4. 超出膨胀层的区域恢复为自由通行区 (0)
        free_zone = (~self.lethal_mask) & (dist_grid > self.robot_radius + self.inflation_radius)
        self.cost_array[free_zone] = self.FREE_SPACE

    def get_cost(self, u: int, v: int) -> int:
        """获取指定栅格的代价值"""
        if not self.is_in_bounds(u, v):
            return self.NO_INFORMATION
        return int(self.cost_array[v, u])

    def get_cost_world(self, x: float, y: float) -> int:
        """获取指定世界坐标处的代价值"""
        u, v = self.world_to_map(x, y)
        return self.get_cost(u, v)

    def is_lethal(self, u: int, v: int) -> bool:
        """判断指定栅格是否不可通行（致命或内切碰撞）"""
        cost = self.get_cost(u, v)
        return cost >= self.INSCRIBED_INFLATED_OBSTACLE

    def is_lethal_world(self, x: float, y: float) -> bool:
        """判断指定世界坐标是否不可通行"""
        u, v = self.world_to_map(x, y)
        return self.is_lethal(u, v)

    @classmethod
    def from_point_cloud(
        cls,
        points: np.ndarray,
        resolution: float = 0.05,
        margin: float = 2.0,
        robot_radius: float = 0.11,
        inflation_radius: float = 0.25,
        z_min: float = 0.06,
        z_max: float = 0.40,
    ) -> "Costmap2D":
        """根据点云自动推导边界并创建自适应 2D 代价地图"""
        if points is None or len(points) == 0:
            return cls(resolution=resolution)

        x_min, x_max = float(np.min(points[:, 0])), float(np.max(points[:, 0]))
        y_min, y_max = float(np.min(points[:, 1])), float(np.max(points[:, 1]))

        origin_x = x_min - margin
        origin_y = y_min - margin
        size_x = (x_max - x_min) + 2.0 * margin
        size_y = (y_max - y_min) + 2.0 * margin

        costmap = cls(
            resolution=resolution,
            size_x=size_x,
            size_y=size_y,
            origin_x=origin_x,
            origin_y=origin_y,
            robot_radius=robot_radius,
            inflation_radius=inflation_radius,
            z_min=z_min,
            z_max=z_max,
        )
        costmap.update_from_point_cloud(points)
        return costmap
