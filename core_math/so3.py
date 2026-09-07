# -*- coding: utf-8 -*-
"""
三维旋转特殊正交群 SO(3) 与其李代数 so(3) 核心数学实现
采用纯 CPU 向量化 NumPy 计算，包含罗德里格斯公式指数/对数映射、小角度极值防护与左雅可比矩阵
"""

import numpy as np


def skew_symmetric(v: np.ndarray) -> np.ndarray:
    """
    计算 3 维向量的反对称矩阵 (Hat 算子 ^)

    参数:
        v: 长度为 3 的向量 [v1, v2, v3]
    返回:
        3x3 反对称矩阵
    """
    v = np.asarray(v, dtype=np.float64).reshape(3)
    return np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ],
        dtype=np.float64,
    )


def vee(mat: np.ndarray) -> np.ndarray:
    """
    从 3x3 反对称矩阵提取向量 (Vee 算子 v)

    参数:
        mat: 3x3 反对称矩阵
    返回:
        长度为 3 的向量
    """
    return np.array([mat[2, 1], mat[0, 2], mat[1, 0]], dtype=np.float64)


class SO3:
    """
    三维特殊正交群 SO(3) 旋转表示
    底层以 3x3 正交旋转矩阵储存
    """

    def __init__(self, rotation_matrix: np.ndarray = None):
        if rotation_matrix is None:
            self.matrix = np.eye(3, dtype=np.float64)
        else:
            self.matrix = np.asarray(rotation_matrix, dtype=np.float64)
            if self.matrix.shape != (3, 3):
                raise ValueError(f"旋转矩阵形状必须为 (3, 3)，实际输入为: {self.matrix.shape}")

    @classmethod
    def identity(cls) -> "SO3":
        """获取单位旋转"""
        return cls(np.eye(3, dtype=np.float64))

    @classmethod
    def exp(cls, omega: np.ndarray) -> "SO3":
        """
        李代数 so(3) 到李群 SO(3) 的指数映射 (Rodrigues 罗德里格斯公式)

        参数:
            omega: 3 维旋转向量 [wx, wy, wz]，模长为旋转角弧度
        返回:
            SO3 对象
        """
        omega = np.asarray(omega, dtype=np.float64).reshape(3)
        theta = np.linalg.norm(omega)
        omega_hat = skew_symmetric(omega)

        if theta < 1e-7:
            # 小角度泰勒展开极值防护
            # sin(theta)/theta ≈ 1 - theta^2 / 6
            # (1 - cos(theta))/theta^2 ≈ 0.5 - theta^2 / 24
            a = 1.0 - (theta**2) / 6.0
            b = 0.5 - (theta**2) / 24.0
        else:
            a = np.sin(theta) / theta
            b = (1.0 - np.cos(theta)) / (theta**2)

        r_mat = np.eye(3, dtype=np.float64) + a * omega_hat + b * (omega_hat @ omega_hat)
        return cls(r_mat)

    def log(self) -> np.ndarray:
        """
        李群 SO(3) 到李代数 so(3) 的对数映射

        返回:
            3 维旋转向量 omega
        """
        trace = np.trace(self.matrix)
        cos_theta = np.clip((trace - 1.0) * 0.5, -1.0, 1.0)
        theta = np.arccos(cos_theta)

        if theta < 1e-7:
            # 小角度近似
            return vee(self.matrix - self.matrix.T) * 0.5

        sin_theta = np.sin(theta)
        if np.abs(sin_theta) < 1e-7:
            # 旋转角接近 pi 的特异情况处理
            diag = np.diagonal(self.matrix)
            k = np.argmax(diag)
            v = np.zeros(3, dtype=np.float64)
            v[k] = np.sqrt(max(0.0, (diag[k] + 1.0) * 0.5))
            for i in range(3):
                if i != k and v[k] > 1e-6:
                    v[i] = (self.matrix[i, k] + self.matrix[k, i]) / (4.0 * v[k])
            return v * theta

        omega_hat = (self.matrix - self.matrix.T) * (theta / (2.0 * sin_theta))
        return vee(omega_hat)

    @classmethod
    def from_quaternion(cls, q: np.ndarray) -> "SO3":
        """
        从四元数构造 SO(3)
        四元数格式为 Hamilton 规范 [x, y, z, w]
        """
        q = np.asarray(q, dtype=np.float64).reshape(4)
        norm = np.linalg.norm(q)
        if norm < 1e-8:
            return cls.identity()
        x, y, z, w = q / norm

        r_mat = np.array(
            [
                [1.0 - 2.0 * (y**2 + z**2), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x**2 + z**2), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x**2 + y**2)],
            ],
            dtype=np.float64,
        )
        return cls(r_mat)

    def to_quaternion(self) -> np.ndarray:
        """
        转换为四元数 [x, y, z, w]
        """
        m = self.matrix
        trace = np.trace(m)
        if trace > 0.0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (m[2, 1] - m[1, 2]) * s
            y = (m[0, 2] - m[2, 0]) * s
            z = (m[1, 0] - m[0, 1]) * s
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = 2.0 * np.sqrt(max(0.0, 1.0 + m[0, 0] - m[1, 1] - m[2, 2]))
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = 2.0 * np.sqrt(max(0.0, 1.0 + m[1, 1] - m[0, 0] - m[2, 2]))
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(max(0.0, 1.0 + m[2, 2] - m[0, 0] - m[1, 1]))
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s

        q = np.array([x, y, z, w], dtype=np.float64)
        return q / np.linalg.norm(q)

    @classmethod
    def from_euler_rpy(cls, roll: float, pitch: float, yaw: float) -> "SO3":
        """
        从外玄欧拉角 (Roll, Pitch, Yaw) 绕定轴 X-Y-Z 顺序构建旋转矩阵
        """
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)

        return cls(rz @ ry @ rx)

    def to_euler_rpy(self) -> tuple[float, float, float]:
        """
        提取欧拉角 (Roll, Pitch, Yaw)，单位为弧度
        """
        m = self.matrix
        pitch = -np.arcsin(np.clip(m[2, 0], -1.0, 1.0))
        if np.abs(np.cos(pitch)) > 1e-6:
            roll = np.arctan2(m[2, 1], m[2, 2])
            yaw = np.arctan2(m[1, 0], m[0, 0])
        else:
            roll = 0.0
            yaw = np.arctan2(-m[0, 1], m[1, 1])
        return float(roll), float(pitch), float(yaw)

    def inverse(self) -> "SO3":
        """正交矩阵的逆等于其转置"""
        return SO3(self.matrix.T)

    def __matmul__(self, other: "SO3") -> "SO3":
        """李群复合 R1 @ R2"""
        if isinstance(other, SO3):
            return SO3(self.matrix @ other.matrix)
        elif isinstance(other, np.ndarray):
            # 支持点或点云旋转 R @ p
            return self.matrix @ other
        raise TypeError(f"不支持与类型 {type(other)} 进行矩阵乘法")

    def act(self, points: np.ndarray) -> np.ndarray:
        """
        作用于空间点或 Nx3 点云
        参数:
            points: 形状为 (3,) 或 (N, 3) 的三维点坐标
        返回:
            变换后的点坐标，形状与输入一致
        """
        pts = np.asarray(points, dtype=np.float64)
        if pts.ndim == 1 and pts.shape[0] == 3:
            return self.matrix @ pts
        elif pts.ndim == 2 and pts.shape[1] == 3:
            return (self.matrix @ pts.T).T
        raise ValueError(f"输入点形状必须为 (3,) 或 (N, 3)，实际为: {pts.shape}")

    @staticmethod
    def left_jacobian(omega: np.ndarray) -> np.ndarray:
        """
        计算 SO(3) 的左雅可比矩阵 J_l(omega)
        """
        omega = np.asarray(omega, dtype=np.float64).reshape(3)
        theta = np.linalg.norm(omega)
        omega_hat = skew_symmetric(omega)

        if theta < 1e-7:
            a = 0.5 - (theta**2) / 24.0
            b = (1.0 / 6.0) - (theta**2) / 120.0
        else:
            a = (1.0 - np.cos(theta)) / (theta**2)
            b = (theta - np.sin(theta)) / (theta**3)

        return np.eye(3, dtype=np.float64) + a * omega_hat + b * (omega_hat @ omega_hat)
