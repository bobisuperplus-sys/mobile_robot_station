# -*- coding: utf-8 -*-
"""
MSH (Mobile Station Host) 主控微服务总线、能力清单与地图持久化自动化测试套件
"""

import os
import shutil
import tempfile
import time
import unittest

import numpy as np

from msh import MSHClient, MSHServer, ServiceHandler, get_robot_capabilities
from navigation import ElevationMap2D, MapStorage, NavigationManager, NavigationState


class TestMSHSystem(unittest.TestCase):
    """MSH 全车能力清单、地图持久化与 JSON-RPC 2.0 通信测试"""

    def setUp(self):
        self.test_dir = tempfile.mkdtemp(prefix="msh_test_")

    def tearDown(self):
        if os.path.exists(self.test_dir):
            shutil.rmtree(self.test_dir)

    def test_capabilities_manifest(self):
        """测试全车功能能力清单结构完整性与必要字段"""
        cap = get_robot_capabilities()
        self.assertIn("system", cap)
        self.assertIn("capabilities", cap)

        sys_info = cap["system"]
        self.assertEqual(sys_info["robot_name"], "TurtleBot3-Burger-Station")
        self.assertEqual(sys_info["communication_protocol"], "TCP JSON-RPC 2.0")
        self.assertIn("NAVIGATION", sys_info["supported_modes"])

        caps = cap["capabilities"]
        # 必须包含的核心能力
        expected_modules = ["teleop", "sensors", "slam", "map_manager", "navigation", "telemetry", "streaming"]
        for mod in expected_modules:
            self.assertIn(mod, caps, f"能力清单中必须声明模块: {mod}")
            self.assertTrue(caps[mod].get("enabled", False))

        # 检查 navigation 能力中的方法与高程特征
        nav_methods = [m["method"] for m in caps["navigation"]["methods"]]
        self.assertIn("nav.navigate_to", nav_methods)
        self.assertIn("nav.cancel_goal", nav_methods)
        self.assertIn("nav.get_status", nav_methods)

    def test_map_storage_persistence(self):
        """测试工业级地图持久化存盘、元数据生成与反序列化加载"""
        # 1. 构造一个测试用高程地图 (20x20, 分辨率 0.1m)
        elev_map = ElevationMap2D(
            resolution=0.1,
            size_x=2.0,
            size_y=2.0,
            origin_x=-1.0,
            origin_y=-1.0,
            robot_radius=0.10,
            inflation_radius=0.20,
        )
        # 生成虚拟地形：缓坡与局部障碍物
        pts = []
        for x in np.linspace(-0.8, 0.8, 15):
            for y in np.linspace(-0.8, 0.8, 15):
                z = 0.05 * (y + 0.8)  # 缓坡
                pts.append([x, y, z])
        pts = np.array(pts, dtype=np.float32)
        elev_map.update_from_point_cloud(pts)

        # 2. 存盘
        map_name = "test_city_map"
        saved = MapStorage.save_map(
            map_name=map_name,
            elevation_map=elev_map,
            point_cloud=pts,
            output_dir=self.test_dir,
            description="自动化单元测试地图",
        )

        self.assertIn("metadata", saved)
        self.assertIn("elevation_data", saved)
        self.assertIn("traversability_img", saved)
        self.assertIn("point_cloud", saved)

        for _, fpath in saved.items():
            self.assertTrue(os.path.exists(fpath), f"存盘文件应存在: {fpath}")

        # 3. 检索地图清单
        maps_list = MapStorage.list_maps(maps_root=self.test_dir)
        self.assertEqual(len(maps_list), 1)
        self.assertEqual(maps_list[0]["name"], map_name)
        self.assertEqual(maps_list[0]["point_count"], len(pts))

        # 4. 反序列化加载并校验数据一致性
        loaded_map, loaded_pts, meta = MapStorage.load_map(map_name, maps_root=self.test_dir)
        self.assertEqual(meta["map_name"], map_name)
        self.assertEqual(loaded_map.nx, elev_map.nx)
        self.assertEqual(loaded_map.ny, elev_map.ny)
        self.assertAlmostEqual(loaded_map.resolution, elev_map.resolution, places=4)

        # 校验数组内容一致性
        np.testing.assert_allclose(loaded_map.elevation_array, elev_map.elevation_array, atol=1e-5)
        np.testing.assert_allclose(loaded_map.cost_array, elev_map.cost_array)
        self.assertIsNotNone(loaded_pts)
        self.assertEqual(len(loaded_pts), len(pts))

        # 5. 删除地图
        deleted = MapStorage.delete_map(map_name, maps_root=self.test_dir)
        self.assertTrue(deleted)
        self.assertEqual(len(MapStorage.list_maps(maps_root=self.test_dir)), 0)

    def test_msh_rpc_server_and_client(self):
        """测试 TCP JSON-RPC 2.0 服务网关请求分发与客户端 SDK 通信"""
        test_port = 19001  # 专用测试端口，避免与默认 9001 冲突
        handler = ServiceHandler(maps_root=self.test_dir)
        server = MSHServer(service_handler=handler, host="127.0.0.1", port=test_port)

        try:
            server.start(blocking=False)
            time.sleep(0.15)  # 等待 socket 监听就绪
            self.assertTrue(server.is_running)

            client = MSHClient(host="127.0.0.1", port=test_port, timeout=2.0)

            # 1. 验证获取全车功能清单
            cap = client.get_capabilities()
            self.assertEqual(cap["system"]["robot_name"], "TurtleBot3-Burger-Station")
            self.assertIn("navigation", cap["capabilities"])

            # 2. 验证获取系统状态
            status = client.get_status()
            self.assertEqual(status["mode"], "MANUAL")
            self.assertFalse(status["is_mapping"])

            # 3. 验证遥控速度指令下发
            drive_res = client.drive(0.25, -0.15)
            self.assertEqual(drive_res["status"], "success")
            self.assertAlmostEqual(drive_res["applied_velocity"]["linear_x"], 0.25)
            self.assertAlmostEqual(drive_res["applied_velocity"]["angular_z"], -0.15)

            # 4. 验证急停制动
            stop_res = client.emergency_stop()
            self.assertEqual(stop_res["status"], "stopped")

            # 5. 验证模式切换与非法参数容错
            res_mode = client.call("system.set_mode", {"mode": "NAVIGATION"})
            self.assertEqual(res_mode["mode"], "NAVIGATION")

            with self.assertRaises(RuntimeError) as ctx:
                client.call("system.set_mode", {"mode": "INVALID_MODE"})
            self.assertIn("RPC Error", str(ctx.exception))

            # 6. 验证未知方法 -32601 错误拦截
            with self.assertRaises(RuntimeError) as ctx:
                client.call("unknown.non_existent_method")
            self.assertIn("-32601", str(ctx.exception))

        finally:
            server.stop()
            time.sleep(0.1)


if __name__ == "__main__":
    unittest.main()
