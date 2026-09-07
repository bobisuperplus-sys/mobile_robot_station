#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CPU 极速 3D 激光雷达光线投射仿真引擎 (Raycasting Lidar Simulator)
基于 MuJoCo 底层原生光线求交接口，实现 16/32 线激光雷达高保真点云生成。
"""

import math
import numpy as np

try:
    import mujoco
except ImportError as err:
    raise ImportError("未检测到 mujoco 模块，请先激活虚拟环境。") from err


class RaycastLidar3D:
    """
    16/32 线多线 3D 激光雷达仿真发生器
    特性:
      - 纯 CPU 运算，无需 GPU/CUDA 参与
      - 预计算局部光线方向查找表 (LUT)，单帧零三角函数开销
      - 自动过滤机器人本体碰撞体，杜绝车身自遮挡噪点
      - 支持注入物理测距高斯白噪声与材质反射强度衰减
      - 结构化输出点云数据 [X, Y, Z, Intensity, Ring]
    """

    def __init__(
        self,
        vertical_channels: int = 16,
        vertical_fov_deg: tuple[float, float] = (-15.0, 15.0),
        horizontal_resolution_deg: float = 3.0,
        min_range: float = 0.15,
        max_range: float = 25.0,
        noise_std: float = 0.01,
    ):
        self.channels = int(vertical_channels)
        self.v_fov_min = math.radians(float(vertical_fov_deg[0]))
        self.v_fov_max = math.radians(float(vertical_fov_deg[1]))
        self.h_res_rad = math.radians(float(horizontal_resolution_deg))
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.noise_std = float(noise_std)

        # 水平方位角点数 (例如 360 / 3.0 = 120 方位角)
        self.h_points = int(math.ceil(2.0 * math.pi / self.h_res_rad))
        self.total_rays = self.channels * self.h_points

        # 预计算局部坐标系下的光线方向查找表与线束标记 (LUT)
        self._init_local_ray_table()

        # 复用静态缓存
        self._geom_id_buffer = np.zeros(1, dtype=np.int32)

    def _init_local_ray_table(self):
        """预计算激光雷达自身坐标系 (Lidar Frame) 下的方向向量查找表"""
        if self.channels > 1:
            pitch_angles = np.linspace(self.v_fov_min, self.v_fov_max, self.channels)
        else:
            pitch_angles = np.array([0.0])

        yaw_angles = np.linspace(0.0, 2.0 * math.pi, self.h_points, endpoint=False)

        rays_local = np.zeros((self.channels, self.h_points, 3), dtype=np.float64)
        rings = np.zeros((self.channels, self.h_points), dtype=np.int32)

        for c_idx, pitch in enumerate(pitch_angles):
            cos_p = math.cos(pitch)
            sin_p = math.sin(pitch)
            for h_idx, yaw in enumerate(yaw_angles):
                # 激光雷达坐标系: X 前向, Y 左向, Z 天向
                rx = cos_p * math.cos(yaw)
                ry = cos_p * math.sin(yaw)
                rz = sin_p
                rays_local[c_idx, h_idx] = [rx, ry, rz]
                rings[c_idx, h_idx] = c_idx

        self.local_ray_dirs = rays_local.reshape(-1, 3)
        self.ray_ring_ids = rings.reshape(-1)

    def scan(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        lidar_site_name: str = "lidar_site",
        robot_body_name: str = "base_link",
        return_world_frame: bool = False,
    ) -> dict[str, np.ndarray]:
        """
        执行单次 3D 光线投射扫描
        :param model: MuJoCo 模型对象
        :param data: MuJoCo 数据对象
        :param lidar_site_name: 激光雷达发射点 Site 名称
        :param robot_body_name: 机器人车体 Body 名称 (用于排除车体自碰撞)
        :param return_world_frame: 若为 True 返回世界坐标点，若为 False 返回雷达自身坐标系点
        :return: 字典包含:
            - 'points': (K, 3) 空间点坐标 [X, Y, Z] (米)
            - 'intensities': (K,) 反射强度 [0, 100]
            - 'rings': (K,) 对应激光束通道编号 [0, channels-1]
            - 'ranges': (K,) 测距真值距离 (米)
            - 'total_rays': 总发射光线数
            - 'valid_count': 击中有效点数
        """
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, lidar_site_name)
        if site_id < 0:
            raise ValueError(f"未找到指定的激光雷达安装站点: {lidar_site_name}")

        body_exclude = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, robot_body_name)

        # 检查旋转矩阵是否已解算，若未初始化则自动触发一次正向解算
        if np.linalg.norm(data.site_xmat[site_id]) < 0.1:
            mujoco.mj_forward(model, data)

        pnt_origin = data.site_xpos[site_id]
        rot_mat = data.site_xmat[site_id].reshape(3, 3)

        # 批量旋转到世界坐标系: dirs_world = dirs_local @ rot_mat.T
        world_ray_dirs = np.dot(self.local_ray_dirs, rot_mat.T)

        geomid = self._geom_id_buffer
        min_r = self.min_range
        max_r = self.max_range
        noise_std = self.noise_std
        local_dirs = self.local_ray_dirs
        ring_ids = self.ray_ring_ids

        valid_points = []
        valid_intensities = []
        valid_rings = []
        valid_ranges = []

        # 极速单线程循环求交
        for i in range(self.total_rays):
            vec = world_ray_dirs[i]
            dist = mujoco.mj_ray(
                model,
                data,
                pnt_origin,
                vec,
                None,
                1,
                body_exclude,
                geomid,
            )

            if min_r <= dist <= max_r:
                noisy_dist = dist + np.random.normal(0.0, noise_std) if noise_std > 0.0 else dist
                if noisy_dist < min_r:
                    noisy_dist = min_r

                if return_world_frame:
                    pt = pnt_origin + noisy_dist * vec
                else:
                    pt = noisy_dist * local_dirs[i]

                # 反射强度衰减模拟: 距离越远强度越弱
                intensity = float(np.clip(100.0 / (1.0 + 0.12 * noisy_dist), 5.0, 100.0))

                valid_points.append(pt)
                valid_intensities.append(intensity)
                valid_rings.append(ring_ids[i])
                valid_ranges.append(dist)

        if len(valid_points) > 0:
            points_arr = np.asarray(valid_points, dtype=np.float32)
            intensities_arr = np.asarray(valid_intensities, dtype=np.float32)
            rings_arr = np.asarray(valid_rings, dtype=np.int16)
            ranges_arr = np.asarray(valid_ranges, dtype=np.float32)
        else:
            points_arr = np.empty((0, 3), dtype=np.float32)
            intensities_arr = np.empty((0,), dtype=np.float32)
            rings_arr = np.empty((0,), dtype=np.int16)
            ranges_arr = np.empty((0,), dtype=np.float32)

        return {
            "points": points_arr,
            "intensities": intensities_arr,
            "rings": rings_arr,
            "ranges": ranges_arr,
            "total_rays": self.total_rays,
            "valid_count": len(valid_points),
        }
