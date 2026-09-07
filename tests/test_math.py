# -*- coding: utf-8 -*-
"""
空间几何代数与李代数核心数学库单元测试用例
"""

import os
import sys
import unittest
import numpy as np

# 导入工程根目录
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from core_math import SO3, SE3, skew_symmetric, vee


class TestCoreMath(unittest.TestCase):
    """验证 SO(3) 与 SE(3) 数学库精度与性质"""

    def test_skew_symmetric_and_vee(self):
        v = np.array([1.2, -3.4, 5.6], dtype=np.float64)
        v_hat = skew_symmetric(v)
        self.assertEqual(v_hat.shape, (3, 3))
        # 反对称性质: v_hat^T = -v_hat
        np.testing.assert_allclose(v_hat.T, -v_hat, atol=1e-12)
        # vee 逆算子
        v_rec = vee(v_hat)
        np.testing.assert_allclose(v, v_rec, atol=1e-12)

    def test_so3_exp_log_roundtrip(self):
        """测试 SO(3) 指数映射与对数映射互逆性"""
        # 1. 普通角度
        omega1 = np.array([0.2, -0.5, 0.8], dtype=np.float64)
        rot1 = SO3.exp(omega1)
        np.testing.assert_allclose(rot1.matrix @ rot1.matrix.T, np.eye(3), atol=1e-10)
        self.assertAlmostEqual(np.linalg.det(rot1.matrix), 1.0, places=9)
        omega1_rec = rot1.log()
        np.testing.assert_allclose(omega1, omega1_rec, atol=1e-10)

        # 2. 极小角度 (< 1e-6)
        omega_small = np.array([1e-8, -2e-8, 3e-8], dtype=np.float64)
        rot_small = SO3.exp(omega_small)
        omega_small_rec = rot_small.log()
        np.testing.assert_allclose(omega_small, omega_small_rec, atol=1e-12)

        # 3. 接近 pi 的大旋转角
        omega_large = np.array([np.pi * 0.999, 0.0, 0.0], dtype=np.float64)
        rot_large = SO3.exp(omega_large)
        omega_large_rec = rot_large.log()
        np.testing.assert_allclose(omega_large, omega_large_rec, atol=1e-4)

    def test_so3_quaternion_and_euler(self):
        """测试四元数与欧拉角转换一致性"""
        roll, pitch, yaw = 0.1, -0.2, 0.5
        rot = SO3.from_euler_rpy(roll, pitch, yaw)
        r_rec, p_rec, y_rec = rot.to_euler_rpy()
        self.assertAlmostEqual(roll, r_rec, places=7)
        self.assertAlmostEqual(pitch, p_rec, places=7)
        self.assertAlmostEqual(yaw, y_rec, places=7)

        # 四元数互转
        q = rot.to_quaternion()
        self.assertEqual(q.shape, (4,))
        self.assertAlmostEqual(np.linalg.norm(q), 1.0, places=9)
        rot_from_q = SO3.from_quaternion(q)
        np.testing.assert_allclose(rot.matrix, rot_from_q.matrix, atol=1e-10)

    def test_so3_point_action(self):
        """测试 SO(3) 作用于三维点与点云"""
        rot = SO3.from_euler_rpy(0.0, 0.0, np.pi / 2.0)  # 绕 Z 轴旋转 90 度
        # 单点测试: (1, 0, 0) -> (0, 1, 0)
        p = np.array([1.0, 0.0, 0.0])
        p_trans = rot.act(p)
        np.testing.assert_allclose(p_trans, [0.0, 1.0, 0.0], atol=1e-10)

        # 点云测试 (N, 3)
        pts = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
        pts_trans = rot.act(pts)
        expected = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        np.testing.assert_allclose(pts_trans, expected, atol=1e-10)

    def test_se3_exp_log_roundtrip(self):
        """测试 SE(3) 指数映射与对数映射互逆性"""
        xi = np.array([1.0, -2.5, 0.8, 0.1, -0.3, 0.6], dtype=np.float64)
        t_se3 = SE3.exp(xi)
        self.assertEqual(t_se3.translation.shape, (3,))
        self.assertEqual(t_se3.rotation.matrix.shape, (3, 3))

        xi_rec = t_se3.log()
        np.testing.assert_allclose(xi, xi_rec, atol=1e-9)

    def test_se3_matrix_and_inverse(self):
        """测试 SE(3) 齐次矩阵变换与求逆"""
        t = SE3.from_xyz_rpy(2.0, 3.0, -1.0, 0.1, 0.2, -0.3)
        mat = t.to_matrix()
        self.assertEqual(mat.shape, (4, 4))
        np.testing.assert_allclose(mat[3, :], [0.0, 0.0, 0.0, 1.0], atol=1e-12)

        # 求逆与复合验证: T @ T^-1 == I
        t_inv = t.inverse()
        t_id = t @ t_inv
        np.testing.assert_allclose(t_id.to_matrix(), np.eye(4), atol=1e-10)

    def test_se3_perturbation_and_point_cloud(self):
        """测试 SE(3) 左微扰与点云空间变换"""
        t = SE3.from_xyz_rpy(1.0, 2.0, 3.0, 0.0, 0.0, 0.0)
        pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float64)
        pts_trans = t.act(pts)
        expected = np.array([[1.0, 2.0, 3.0], [2.0, 2.0, 3.0]], dtype=np.float64)
        np.testing.assert_allclose(pts_trans, expected, atol=1e-10)

        # 纯平移左微扰更新
        delta_trans = np.array([0.1, -0.2, 0.3, 0.0, 0.0, 0.0])
        t_pure_trans = t.apply_left_perturbation(delta_trans)
        np.testing.assert_allclose(t_pure_trans.translation, [1.1, 1.8, 3.3], atol=1e-10)

        # 包含旋转与平移的通用左微扰
        delta_xi = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.05])
        t_perturbed = t.apply_left_perturbation(delta_xi)
        expected_t = SE3.exp(delta_xi) @ t
        np.testing.assert_allclose(t_perturbed.to_matrix(), expected_t.to_matrix(), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
