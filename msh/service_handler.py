# -*- coding: utf-8 -*-
"""
MSH (Mobile Station Host) 服务请求分发中心与业务中枢 (Service Handler)

核心职责：
1. 集中实现 JSON-RPC 2.0 协议中声明的所有方法业务逻辑；
2. 调度仿真底盘 (Simulation)、驱动器 (Driver)、建图 (SLAM) 与导航 (NavigationManager)；
3. 支持动态查询全车功能能力清单 (system.get_capabilities)；
4. 保证多线程通信环境下的状态一致性与异常防护。
"""

import math
import os
import threading
import time
from typing import Any, Dict, List, Optional

import numpy as np

from msh.capabilities import get_robot_capabilities
from navigation.elevation_map import ElevationMap2D
from navigation.map_storage import MapStorage
from navigation.navigation_manager import NavigationManager, NavigationState
from simulation.robot_driver import TurtleBot3BurgerDriver
from simulation.world_sim import UrbanWorldSimulation


class ServiceHandler:
    """MSH 核心业务调度中心"""

    def __init__(
        self,
        sim: Optional[UrbanWorldSimulation] = None,
        driver: Optional[TurtleBot3BurgerDriver] = None,
        navigation_mgr: Optional[NavigationManager] = None,
        maps_root: str = "maps",
    ) -> None:
        self.sim = sim
        self.driver = driver or TurtleBot3BurgerDriver(wheel_radius=0.033, wheel_base=0.160, max_v=0.60, max_w=3.5)
        self.navigation_mgr = navigation_mgr
        self.maps_root = maps_root

        # 内部状态
        self.mode = "MANUAL"
        self._lock = threading.Lock()
        self.is_mapping = False
        self._cached_pointcloud: Optional[np.ndarray] = None

        # 如果传入了仿真环境但未传入导航管理器，自动初始化
        if self.sim is not None and self.navigation_mgr is None:
            self.navigation_mgr = NavigationManager(self.sim, self.driver)

    def dispatch(self, method: str, params: Any) -> Any:
        """
        根据方法名称将 RPC 请求分发至对应的业务处理函数

        参数:
            method: 方法名 (如 "system.get_capabilities", "teleop.drive")
            params: 参数字典或列表
        返回:
            执行结果字典或值
        """
        handlers = {
            # 1. 系统与能力查询
            "system.get_capabilities": self.handle_get_capabilities,
            "system.get_status": self.handle_get_status,
            "system.set_mode": self.handle_set_mode,

            # 2. 底盘驱动遥控
            "teleop.drive": self.handle_teleop_drive,
            "teleop.emergency_stop": self.handle_teleop_stop,

            # 3. 3D SLAM 建图
            "slam.start_mapping": self.handle_slam_start,
            "slam.stop_mapping": self.handle_slam_stop,
            "slam.save_map": self.handle_slam_save_map,
            "slam.get_map_stats": self.handle_slam_get_stats,

            # 4. 地图持久化管理
            "map.list_maps": self.handle_map_list,
            "map.load_map": self.handle_map_load,
            "map.delete_map": self.handle_map_delete,

            # 5. 自主导航
            "nav.navigate_to": self.handle_nav_navigate_to,
            "nav.cancel_goal": self.handle_nav_cancel,
            "nav.get_status": self.handle_nav_get_status,
            "nav.get_trajectory": self.handle_nav_get_trajectory,

            # 6. 遥测状态
            "telemetry.get_pose": self.handle_telemetry_get_pose,
            "telemetry.get_full_status": self.handle_telemetry_get_full,
        }

        handler = handlers.get(method)
        if handler is None:
            raise KeyError(f"未知的 RPC 方法: {method}")

        if isinstance(params, dict):
            return handler(**params)
        elif isinstance(params, (list, tuple)):
            return handler(*params)
        elif params is None:
            return handler()
        else:
            return handler(params)

    # ---------------- 1. 系统与能力 ----------------
    def handle_get_capabilities(self) -> Dict[str, Any]:
        """获取全车能力清单"""
        manifest = get_robot_capabilities()
        manifest["system"]["current_mode"] = self.mode
        manifest["system"]["is_sim_active"] = (self.sim is not None)
        return manifest

    def handle_get_status(self) -> Dict[str, Any]:
        """获取系统当前综合运行状态"""
        with self._lock:
            status = {
                "mode": self.mode,
                "is_mapping": self.is_mapping,
                "timestamp": time.time(),
            }
            if self.navigation_mgr is not None:
                status["navigation_state"] = self.navigation_mgr.state.value
            else:
                status["navigation_state"] = "UNAVAILABLE"
            return status

    def handle_set_mode(self, mode: str) -> Dict[str, Any]:
        """切换运行模式"""
        mode = mode.upper().strip()
        valid_modes = ["MANUAL", "MAPPING", "NAVIGATION", "ESTOP"]
        if mode not in valid_modes:
            raise ValueError(f"无效模式: {mode}，合法值为: {valid_modes}")
        with self._lock:
            self.mode = mode
            if mode == "ESTOP":
                self.handle_teleop_stop()
        return {"mode": self.mode, "status": "success"}

    # ---------------- 2. 底盘动力学遥控 ----------------
    def handle_teleop_drive(self, linear_x: float = 0.0, angular_z: float = 0.0) -> Dict[str, Any]:
        """遥控小车运动"""
        with self._lock:
            if self.mode == "ESTOP":
                return {"status": "rejected", "reason": "系统处于急停状态 (ESTOP)"}
            self.driver.set_velocity(float(linear_x), float(angular_z))
            if self.sim is not None:
                w_left, w_right = self.driver.get_wheel_angular_velocities()
                self.sim.apply_wheel_controls(w_left, w_right)
                self.sim.step()
        return {
            "status": "success",
            "applied_velocity": {"linear_x": float(linear_x), "angular_z": float(angular_z)},
        }

    def handle_teleop_stop(self) -> Dict[str, Any]:
        """紧急刹车制动"""
        with self._lock:
            self.driver.emergency_stop()
            if self.sim is not None:
                self.sim.apply_wheel_controls(0.0, 0.0)
                self.sim.step()
            if self.navigation_mgr is not None:
                self.navigation_mgr.emergency_stop()
        return {"status": "stopped"}

    # ---------------- 3. SLAM 建图 ----------------
    def handle_slam_start(self) -> Dict[str, Any]:
        """启动建图模式"""
        with self._lock:
            self.mode = "MAPPING"
            self.is_mapping = True
        return {"status": "mapping_started", "mode": self.mode}

    def handle_slam_stop(self) -> Dict[str, Any]:
        """停止建图模式"""
        with self._lock:
            self.is_mapping = False
            self.mode = "MANUAL"
        return {"status": "mapping_stopped", "mode": self.mode}

    def handle_slam_save_map(self, map_name: str, description: str = "3D SLAM Map") -> Dict[str, Any]:
        """保存当前建图产物"""
        if self.sim is None:
            raise RuntimeError("仿真环境未就绪，无法采集点云建图！")

        # 采样当前环境点云构建 2.5D 高程图并存盘
        pts = self.sim.get_lidar_pointcloud(return_world_frame=True)["points"]
        elev_map = ElevationMap2D(resolution=0.05, size_x=24.0, size_y=24.0, origin_x=-12.0, origin_y=-12.0)
        elev_map.update_from_point_cloud(pts)

        saved = MapStorage.save_map(
            map_name=map_name,
            elevation_map=elev_map,
            point_cloud=pts,
            output_dir=self.maps_root,
            description=description,
        )
        return {"status": "map_saved", "map_name": map_name, "files": saved}

    def handle_slam_get_stats(self) -> Dict[str, Any]:
        """获取 SLAM 统计数据"""
        pts_count = 0
        if self.sim is not None:
            scan = self.sim.get_lidar_pointcloud()
            pts_count = len(scan["points"])
        return {
            "is_mapping": self.is_mapping,
            "current_scan_points": pts_count,
        }

    # ---------------- 4. 地图管理 ----------------
    def handle_map_list(self) -> Dict[str, Any]:
        """列举存储库中的所有地图"""
        maps = MapStorage.list_maps(maps_root=self.maps_root)
        return {"count": len(maps), "maps": maps}

    def handle_map_load(self, map_name: str) -> Dict[str, Any]:
        """加载指定地图至导航系统"""
        if self.navigation_mgr is None:
            raise RuntimeError("导航系统未就绪！")
        success = self.navigation_mgr.load_map_from_storage(map_name, maps_root=self.maps_root)
        if not success:
            raise FileNotFoundError(f"加载地图失败: {map_name}")
        return {"status": "loaded", "map_name": map_name}

    def handle_map_delete(self, map_name: str) -> Dict[str, Any]:
        """删除指定地图"""
        success = MapStorage.delete_map(map_name, maps_root=self.maps_root)
        return {"status": "deleted" if success else "not_found", "map_name": map_name}

    # ---------------- 5. 自主导航 ----------------
    def handle_nav_navigate_to(
        self,
        goal_x: float,
        goal_y: float,
        goal_yaw: Optional[float] = None,
        via_points: Optional[List[List[float]]] = None,
    ) -> Dict[str, Any]:
        """下发导航目标点"""
        if self.navigation_mgr is None:
            raise RuntimeError("导航系统未就绪！")

        v_points = None
        if via_points:
            v_points = [(float(p[0]), float(p[1])) for p in via_points]

        with self._lock:
            self.mode = "NAVIGATION"

        success = self.navigation_mgr.navigate_to(
            goal_x=float(goal_x),
            goal_y=float(goal_y),
            goal_yaw=float(goal_yaw) if goal_yaw is not None else None,
            via_points=v_points,
            async_run=True,
        )

        return {
            "status": "accepted" if success else "planning_failed",
            "goal": [float(goal_x), float(goal_y)],
            "mode": self.mode,
        }

    def handle_nav_cancel(self) -> Dict[str, Any]:
        """取消导航任务"""
        if self.navigation_mgr is not None:
            self.navigation_mgr.cancel_goal()
        with self._lock:
            self.mode = "MANUAL"
        return {"status": "cancelled", "mode": self.mode}

    def handle_nav_get_status(self) -> Dict[str, Any]:
        """获取导航状态机快照"""
        if self.navigation_mgr is None:
            return {"state": "UNAVAILABLE"}
        return {
            "state": self.navigation_mgr.state.value,
            "goal": [self.navigation_mgr.goal_x, self.navigation_mgr.goal_y],
            "distance_to_goal": float(self.navigation_mgr.dist_to_goal),
        }

    def handle_nav_get_trajectory(self) -> Dict[str, Any]:
        """获取导航路径与轨迹"""
        if self.navigation_mgr is None:
            return {"global_path": [], "robot_trajectory": []}
        return {
            "global_path": self.navigation_mgr.global_path,
            "robot_trajectory": self.navigation_mgr.robot_trajectory_2d,
        }

    # ---------------- 6. 遥测状态 ----------------
    def handle_telemetry_get_pose(self) -> Dict[str, Any]:
        """获取小车当前瞬时位姿"""
        if self.sim is None:
            return {"position": [0.0, 0.0, 0.0], "yaw": 0.0}
        pos = self.sim.get_robot_position()
        yaw = self.sim.get_robot_yaw()
        return {
            "position": [float(pos[0]), float(pos[1]), float(pos[2])],
            "yaw_rad": float(yaw),
            "yaw_deg": math.degrees(yaw),
        }

    def handle_telemetry_get_full(self) -> Dict[str, Any]:
        """获取全量动力学遥测快照"""
        pose = self.handle_telemetry_get_pose()
        nav_telemetry = self.navigation_mgr.get_telemetry() if self.navigation_mgr else {}
        return {
            "timestamp": time.time(),
            "mode": self.mode,
            "pose": pose,
            "navigation": nav_telemetry,
        }
