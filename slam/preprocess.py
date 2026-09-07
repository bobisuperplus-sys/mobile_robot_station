# -*- coding: utf-8 -*-
"""
3D 激光雷达点云预处理与体素降采样模块
基于纯 CPU 极速 NumPy 实现，包含有效视距截断、自遮挡剔除、体素空间质心聚合与地面切片粗提取
"""

import numpy as np


class PointcloudPreprocessor:
    """
    激光点云预处理器
    """

    def __init__(
        self,
        voxel_size: float = 0.08,
        min_range: float = 0.15,
        max_range: float = 20.0,
        ground_z_thresh: float = 0.04,
    ):
        """
        参数:
            voxel_size: 体素栅格边长 (单位: 米)
            min_range: 最小有效探测距离 (米，过滤小车本体自遮挡)
            max_range: 最大有效探测距离 (米)
            ground_z_thresh: 地面判定高度阈值 (在底盘坐标系下，单位: 米)
        """
        self.voxel_size = float(voxel_size)
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.ground_z_thresh = float(ground_z_thresh)

    def range_filter(self, points: np.ndarray) -> np.ndarray:
        """
        有效探测距离滤波，剔除过近的车体自遮挡点与过远的发散噪点

        参数:
            points: (N, 3) 点云
        返回:
            (M, 3) 过滤后的点云
        """
        if len(points) == 0:
            return np.empty((0, 3), dtype=np.float64)

        pts = np.asarray(points, dtype=np.float64)
        dist_sq = np.sum(pts**2, axis=1)
        min_sq = self.min_range**2
        max_sq = self.max_range**2

        valid_mask = (dist_sq >= min_sq) & (dist_sq <= max_sq)
        return pts[valid_mask]

    def voxel_downsample(self, points: np.ndarray, voxel_size: float = None) -> np.ndarray:
        """
        基于三维空间体素网格的质心聚合降采样

        参数:
            points: (N, 3) 点云
            voxel_size: 可选覆盖默认体素尺寸
        返回:
            (K, 3) 降采样后的体素质心点云
        """
        if len(points) == 0:
            return np.empty((0, 3), dtype=np.float64)

        pts = np.asarray(points, dtype=np.float64)
        v_size = self.voxel_size if voxel_size is None else float(voxel_size)

        # 量化为整型体素坐标
        voxel_coords = np.floor(pts / v_size).astype(np.int64)

        # 使用字典按体素三元组快速分组并求质心
        # 对于 1500~3000 点，纯字典遍历在 Python 中仅需 1~2ms
        voxel_dict: dict[tuple[int, int, int], list[np.ndarray]] = {}
        for i in range(len(pts)):
            key = (int(voxel_coords[i, 0]), int(voxel_coords[i, 1]), int(voxel_coords[i, 2]))
            if key in voxel_dict:
                voxel_dict[key].append(pts[i])
            else:
                voxel_dict[key] = [pts[i]]

        # 计算每个非空体素内的几何质心 (Centroid)
        downsampled = np.array([np.mean(cluster, axis=0) for cluster in voxel_dict.values()], dtype=np.float64)
        return downsampled

    def split_ground_and_obstacles(
        self, points: np.ndarray, ground_z_thresh: float = None
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        粗分类地面点与立体障碍物点 (基于车身局部坐标系高度)

        参数:
            points: (N, 3) 点云
            ground_z_thresh: 地面高度阈值 (相对于底盘中心)
        返回:
            (ground_points, obstacle_points)
        """
        if len(points) == 0:
            empty = np.empty((0, 3), dtype=np.float64)
            return empty, empty

        pts = np.asarray(points, dtype=np.float64)
        thresh = self.ground_z_thresh if ground_z_thresh is None else float(ground_z_thresh)

        is_ground = pts[:, 2] <= thresh
        return pts[is_ground], pts[~is_ground]

    def process(self, raw_points: np.ndarray) -> np.ndarray:
        """
        全套流水线：测距过滤 -> 体素网格降采样

        参数:
            raw_points: (N, 3) 原始点云
        返回:
            (M, 3) 预处理后的优质特征点云
        """
        filtered = self.range_filter(raw_points)
        return self.voxel_downsample(filtered)
