# -*- coding: utf-8 -*-
"""
3D 激光惯导紧耦合里程计 (LIO Odometry) 与增量式空间建图核心
采用点到面高斯-牛顿优化算法、cKDTree 空间最近邻索引与轮式移动底盘抗退化约束
"""

import numpy as np
from scipy.spatial import cKDTree

from core_math.so3 import SO3, skew_symmetric
from core_math.se3 import SE3
from slam.preprocess import PointcloudPreprocessor
from slam.imu_tracker import IMUKinematicTracker


class LIOOdometry:
    """
    3D 激光雷达与惯导紧耦合里程计
    """

    def __init__(
        self,
        voxel_size: float = 0.08,
        map_voxel_size: float = 0.10,
        max_correspondence_dist: float = 0.60,
        max_iterations: int = 8,
        convergence_thresh: float = 1e-4,
        lidar_extrinsic_trans: list[float] = None,
    ):
        """
        参数:
            voxel_size: 当前帧点云降采样体素大小 (米)
            map_voxel_size: 全局/局部地图降采样体素大小 (米)
            max_correspondence_dist: 点到面关联最大距离阈值 (米)
            max_iterations: 高斯-牛顿最大迭代步数
            convergence_thresh: 迭代微扰收敛阈值 (模长小于此值即认为收敛)
            lidar_extrinsic_trans: 雷达相对于小车底盘的平移安装外参 [x, y, z]
        """
        self.preprocessor = PointcloudPreprocessor(voxel_size=voxel_size)
        self.map_voxel_size = float(map_voxel_size)
        self.max_corr_dist = float(max_correspondence_dist)
        self.max_iterations = int(max_iterations)
        self.convergence_thresh = float(convergence_thresh)

        # 激光雷达相对于底盘质心的外参 T_body_lidar (默认升高 0.172m)
        ext_t = [0.0, 0.0, 0.172] if lidar_extrinsic_trans is None else lidar_extrinsic_trans
        self.t_body_lidar = SE3(SO3.identity(), np.array(ext_t, dtype=np.float64))

        # 运动学积分器
        self.imu_tracker = IMUKinematicTracker()

        # 状态量: 底盘在世界系下的位姿 T_world_body
        self.current_pose = SE3.identity()

        # 全局与局部地图点云 (世界坐标系)
        self.global_map = np.empty((0, 3), dtype=np.float64)
        self.local_kdtree: cKDTree = None

        # 历史轨迹与统计
        self.trajectory: list[SE3] = []
        self.frame_count = 0

    def reset(self, initial_pose: SE3 = None):
        """重置里程计与地图"""
        self.current_pose = SE3.identity() if initial_pose is None else initial_pose
        self.imu_tracker.reset(self.current_pose)
        self.global_map = np.empty((0, 3), dtype=np.float64)
        self.local_kdtree = None
        self.trajectory = [self.current_pose]
        self.frame_count = 0

    def predict_with_imu(self, gyro: np.ndarray, accel: np.ndarray, dt: float) -> SE3:
        """
        利用高频 IMU 数据推进运动先验位姿

        参数:
            gyro: 角速度 (rad/s)
            accel: 线加速度 (m/s^2)
            dt: 时间间隔 (秒)
        返回:
            更新后的先验位姿
        """
        self.current_pose = self.imu_tracker.propagate(gyro, accel, dt)
        return self.current_pose

    def _estimate_normals_and_residuals(
        self,
        transformed_pts: np.ndarray,
        source_pts_body: np.ndarray,
        rot_mat: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray, float]:
        """
        在局部地图中检索最近邻点，通过 PCA 拟合切平面并计算雅可比与残差

        返回:
            (J_matrix, r_vector, mean_residual)
        """
        if self.local_kdtree is None or len(self.global_map) < 10:
            return np.empty((0, 6)), np.empty((0,)), 0.0

        # 查询 5 个最近邻点
        dists, indices = self.local_kdtree.query(transformed_pts, k=5, workers=2)

        j_rows = []
        r_vals = []

        for i in range(len(transformed_pts)):
            if dists[i, 0] > self.max_corr_dist:
                continue

            neighbor_pts = self.global_map[indices[i]]
            centroid = np.mean(neighbor_pts, axis=0)

            # 局部协方差矩阵 PCA 特征分解
            diff = neighbor_pts - centroid
            cov = (diff.T @ diff) / 5.0
            eigen_vals, eigen_vecs = np.linalg.eigh(cov)

            # 最小特征值对应的主方向为切平面法向量
            if eigen_vals[0] / (eigen_vals[2] + 1e-8) > 0.3:
                # 平面度不足 (可能为杂乱立柱或离群点)，放弃该约束
                continue

            normal = eigen_vecs[:, 0]
            # 保证法向量朝向观测源
            if np.dot(normal, transformed_pts[i] - self.current_pose.translation) > 0.0:
                normal = -normal

            # 点到面垂直残差: r = n^T * (p_world - centroid)
            r = float(np.dot(normal, transformed_pts[i] - centroid))

            # 雅可比矩阵求导: J = [n^T, -n^T * (R * p_body)^]
            # 对平移 rho 与旋转 phi 求导
            p_rot = rot_mat @ source_pts_body[i]
            p_rot_skew = skew_symmetric(p_rot)
            j_rot = -normal @ p_rot_skew

            j_i = np.hstack([normal, j_rot])
            j_rows.append(j_i)
            r_vals.append(r)

        if len(j_rows) < 15:
            return np.empty((0, 6)), np.empty((0,)), 0.0

        return np.array(j_rows, dtype=np.float64), np.array(r_vals, dtype=np.float64), float(np.mean(np.abs(r_vals)))

    def register_frame(self, raw_lidar_points: np.ndarray) -> tuple[SE3, float]:
        """
        执行单帧点云配准、位姿优化与增量地图更新

        参数:
            raw_lidar_points: (N, 3) 激光雷达坐标系下的原始点云
        返回:
            (optimized_pose, iteration_residual)
        """
        self.frame_count += 1

        # 1. 点云预处理与体素降采样
        downsampled_lidar = self.preprocessor.process(raw_lidar_points)
        if len(downsampled_lidar) < 20:
            return self.current_pose, 0.0

        # 将点云从雷达坐标系变换至小车底盘坐标系
        pts_body = self.t_body_lidar.act(downsampled_lidar)

        # 2. 如果是首帧，直接将点云作为地图初始化锚点
        if len(self.global_map) < 30:
            self.global_map = self.current_pose.act(pts_body)
            self.local_kdtree = cKDTree(self.global_map)
            self.trajectory.append(self.current_pose)
            return self.current_pose, 0.0

        # 3. 点到面高斯-牛顿迭代配准 (Gauss-Newton Optimization)
        pose_est = self.current_pose
        final_residual = 0.0

        for it in range(self.max_iterations):
            # 将当前帧底盘点云按当前位姿估计投射到世界坐标系
            pts_world = pose_est.act(pts_body)
            r_mat = pose_est.rotation.matrix

            j_mat, r_vec, mean_res = self._estimate_normals_and_residuals(pts_world, pts_body, r_mat)
            final_residual = mean_res

            if len(j_mat) < 15:
                break

            # 构建高斯-牛顿正规方程: (H + lambda * I) * delta_xi = b
            h_mat = j_mat.T @ j_mat
            b_vec = -j_mat.T @ r_vec

            # 注入轮式差速底盘先验正则化 (抑制平坦地面的重力与侧偏退化)
            # 对 Z 轴平移与 Roll/Pitch 旋转给予轻度阻尼约束
            diag_damping = np.array([1e-3, 1e-3, 0.1, 0.1, 0.1, 1e-3], dtype=np.float64)
            h_damped = h_mat + np.diag(diag_damping)

            try:
                delta_xi = np.linalg.solve(h_damped, b_vec)
            except np.linalg.LinAlgError:
                break

            # 左微扰迭代更新位姿: T <- exp(delta_xi) * T
            pose_est = pose_est.apply_left_perturbation(delta_xi)

            if np.linalg.norm(delta_xi) < self.convergence_thresh:
                break

        # 4. 位姿校准与 IMU 先验状态对齐
        self.current_pose = pose_est
        self.imu_tracker.set_pose(self.current_pose)
        self.trajectory.append(self.current_pose)

        # 5. 增量式将新帧融入全局地图
        new_world_pts = self.current_pose.act(pts_body)
        self._update_map(new_world_pts)

        return self.current_pose, final_residual

    def _update_map(self, new_points_world: np.ndarray):
        """
        动态融合新点云并执行全局体素降采样，维护树形索引
        """
        # 合并新点云
        combined = np.vstack([self.global_map, new_points_world])

        # 全局地图体素降采样以保持点云密度均匀、限制内存开销
        self.global_map = self.preprocessor.voxel_downsample(combined, voxel_size=self.map_voxel_size)

        # 更新 k-d 树加速结构 (重建 5000~10000 点仅需 1~2ms)
        self.local_kdtree = cKDTree(self.global_map)

    def get_trajectory_array(self) -> np.ndarray:
        """提取历史轨迹三维坐标 (M, 3)"""
        if not self.trajectory:
            return np.empty((0, 3), dtype=np.float64)
        return np.array([pose.translation for pose in self.trajectory], dtype=np.float64)

    def get_map_points(self) -> np.ndarray:
        """获取当前全局 3D 点云地图"""
        return self.global_map.copy()
