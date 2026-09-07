# -*- coding: utf-8 -*-
"""
三维特殊欧氏群 SE(3) 与其李代数 se(3) 刚体变换核心数学实现
采用纯 CPU 向量化 NumPy 计算，支持 4x4 齐次矩阵、点云仿射变换与切空间六维微扰更新
"""

import numpy as np
from core_math.so3 import SO3, skew_symmetric


class SE3:
    """
    三维特殊欧氏群 SE(3) 刚体位姿表示
    包含旋转 R (SO3) 与平移 t (3维向量)
    """

    def __init__(self, rotation: SO3 = None, translation: np.ndarray = None):
        if rotation is None:
            self.rotation = SO3.identity()
        elif isinstance(rotation, SO3):
            self.rotation = rotation
        elif isinstance(rotation, np.ndarray):
            self.rotation = SO3(rotation)
        else:
            raise TypeError(f"无效的旋转类型: {type(rotation)}")

        if translation is None:
            self.translation = np.zeros(3, dtype=np.float64)
        else:
            self.translation = np.asarray(translation, dtype=np.float64).reshape(3)

    @classmethod
    def identity(cls) -> "SE3":
        """获取单位刚体变换"""
        return cls(SO3.identity(), np.zeros(3, dtype=np.float64))

    @classmethod
    def from_matrix(cls, matrix4x4: np.ndarray) -> "SE3":
        """从 4x4 齐次变换矩阵构建 SE3"""
        mat = np.asarray(matrix4x4, dtype=np.float64)
        if mat.shape != (4, 4):
            raise ValueError(f"变换矩阵形状必须为 (4, 4)，实际为: {mat.shape}")
        return cls(SO3(mat[:3, :3]), mat[:3, 3])

    @classmethod
    def from_xyz_rpy(cls, x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> "SE3":
        """从平移 (x, y, z) 与外玄欧拉角 (Roll, Pitch, Yaw) 构建"""
        rot = SO3.from_euler_rpy(roll, pitch, yaw)
        trans = np.array([x, y, z], dtype=np.float64)
        return cls(rot, trans)

    @classmethod
    def exp(cls, xi: np.ndarray) -> "SE3":
        """
        李代数 se(3) 到李群 SE(3) 的指数映射

        参数:
            xi: 6 维切空间李代数向量 [rho, phi]，前 3 维为平移相关，后 3 维为旋转向量
        返回:
            SE3 对象
        """
        xi = np.asarray(xi, dtype=np.float64).reshape(6)
        rho = xi[:3]
        phi = xi[3:]

        rot = SO3.exp(phi)
        jl = SO3.left_jacobian(phi)
        trans = jl @ rho
        return cls(rot, trans)

    def log(self) -> np.ndarray:
        """
        李群 SE(3) 到李代数 se(3) 的对数映射

        返回:
            6 维李代数向量 [rho, phi]
        """
        phi = self.rotation.log()
        theta = np.linalg.norm(phi)
        phi_hat = skew_symmetric(phi)

        if theta < 1e-7:
            # J_l^-1 的小角度一阶展开
            # J_l^-1 ≈ I - 0.5 * phi_hat + 1/12 * phi_hat^2
            jl_inv = np.eye(3, dtype=np.float64) - 0.5 * phi_hat + (1.0 / 12.0) * (phi_hat @ phi_hat)
        else:
            half_theta = 0.5 * theta
            cot_half_theta = 1.0 / np.tan(half_theta)
            a = 1.0 - half_theta * cot_half_theta
            jl_inv = (
                np.eye(3, dtype=np.float64)
                - 0.5 * phi_hat
                + (a / (theta**2)) * (phi_hat @ phi_hat)
            )

        rho = jl_inv @ self.translation
        return np.concatenate([rho, phi])

    def to_matrix(self) -> np.ndarray:
        """返回 4x4 齐次变换矩阵"""
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = self.rotation.matrix
        mat[:3, 3] = self.translation
        return mat

    def inverse(self) -> "SE3":
        """
        计算刚体变换的逆 T^-1 = [R^T, -R^T * t]
        """
        r_inv = self.rotation.inverse()
        t_inv = -r_inv.act(self.translation)
        return SE3(r_inv, t_inv)

    def __matmul__(self, other: "SE3") -> "SE3":
        """
        李群复合 T1 @ T2
        """
        if isinstance(other, SE3):
            new_rot = self.rotation @ other.rotation
            new_trans = self.rotation.act(other.translation) + self.translation
            return SE3(new_rot, new_trans)
        raise TypeError(f"不支持与类型 {type(other)} 进行矩阵复合")

    def act(self, points: np.ndarray) -> np.ndarray:
        """
        将位姿变换作用于 3 维点或 (N, 3) 点云
        公式: p' = R * p + t

        参数:
            points: (3,) 或 (N, 3) 空间点坐标
        返回:
            变换后的坐标，形状与输入一致
        """
        pts = np.asarray(points, dtype=np.float64)
        if pts.ndim == 1 and pts.shape[0] == 3:
            return self.rotation.act(pts) + self.translation
        elif pts.ndim == 2 and pts.shape[1] == 3:
            return self.rotation.act(pts) + self.translation[None, :]
        raise ValueError(f"输入点云形状必须为 (3,) 或 (N, 3)，实际为: {pts.shape}")

    def apply_left_perturbation(self, delta_xi: np.ndarray) -> "SE3":
        """
        左微扰更新: T_new = exp(delta_xi) @ T_old
        参数:
            delta_xi: 6 维微扰向量 [delta_rho, delta_phi]
        """
        delta_t = SE3.exp(delta_xi)
        return delta_t @ self

    def to_xyz_rpy(self) -> tuple[float, float, float, float, float, float]:
        """
        提取六自由度位姿 (X, Y, Z, Roll, Pitch, Yaw)
        """
        x, y, z = self.translation
        roll, pitch, yaw = self.rotation.to_euler_rpy()
        return float(x), float(y), float(z), float(roll), float(pitch), float(yaw)
