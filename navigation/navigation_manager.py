# -*- coding: utf-8 -*-
"""
高内聚移动机器人自主导航协调管理器 (Navigation Manager)

核心职责：
1. 导航有限状态机 (FSM): IDLE, INITIALIZING, PLANNING, NAVIGATING, ARRIVED, BLOCKED, EMERGENCY_STOP;
2. 全流程链路整合: 先验地图加载 -> 2.5D 高程感知 -> A* 全局立体寻路 -> DWA 局部避障 -> 逆运动学轮速下发;
3. 异步并发控制: 后台非阻塞控制循环，支持线程安全的目标下发、暂停、取消、急停与高频遥测轮询;
4. 动态越障与高程适应: 实时监测车身爬坡俯仰角 (Pitch Angle)、高程 Z 坐标与目标点收敛距离。
"""

from enum import Enum
import math
import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from navigation.elevation_map import ElevationMap2D
from navigation.global_planner import GlobalPlannerAStar
from navigation.local_planner import LocalPlannerDWA
from navigation.map_storage import MapStorage
from simulation.robot_driver import TurtleBot3BurgerDriver
from simulation.world_sim import UrbanWorldSimulation


class NavigationState(str, Enum):
    """自主导航有限状态机状态枚举"""
    IDLE = "IDLE"                         # 空闲待命
    INITIALIZING = "INITIALIZING"         # 地图与传感器准备中
    PLANNING = "PLANNING"                 # A* 全局路径解算中
    NAVIGATING = "NAVIGATING"             # DWA 避障巡航与爬坡中
    ARRIVED = "ARRIVED"                   # 精准抵达目标并驻车
    BLOCKED = "BLOCKED"                   # 前方死锁受阻
    CANCELLED = "CANCELLED"               # 任务被取消
    EMERGENCY_STOP = "EMERGENCY_STOP"     # 紧急制动刹车


class NavigationManager:
    """自主导航协调管理器，连接感知地图、路径规划与底盘物理动力学"""

    def __init__(
        self,
        sim: UrbanWorldSimulation,
        driver: Optional[TurtleBot3BurgerDriver] = None,
        elevation_map: Optional[ElevationMap2D] = None,
        dt_control: float = 0.1,
    ) -> None:
        self.sim = sim
        self.driver = driver or TurtleBot3BurgerDriver(
            wheel_radius=0.033,
            wheel_base=0.160,
            max_v=0.55,
            max_w=3.5,
        )
        self.elevation_map = elevation_map
        self.dt_control = float(dt_control)

        # 规划器组件
        self.global_planner: Optional[GlobalPlannerAStar] = None
        self.local_planner: Optional[LocalPlannerDWA] = None
        if self.elevation_map is not None:
            self._init_planners()

        # 状态机与线程安全锁
        self.state = NavigationState.IDLE
        self._lock = threading.Lock()
        self._stop_requested = False
        self._worker_thread: Optional[threading.Thread] = None

        # 导航任务变量
        self.goal_x: float = 0.0
        self.goal_y: float = 0.0
        self.goal_yaw: Optional[float] = None
        self.global_path: List[Tuple[float, float]] = []
        self.current_subgoal: Tuple[float, float] = (0.0, 0.0)

        # 遥测历史数据缓存
        self.robot_trajectory_2d: List[Tuple[float, float]] = []
        self.robot_trajectory_3d: List[Tuple[float, float, float]] = []
        self.time_series: List[float] = []
        self.vx_history: List[float] = []
        self.vw_history: List[float] = []
        self.pitch_history: List[float] = []
        self.dist_history: List[float] = []

        self.current_vx: float = 0.0
        self.current_vw: float = 0.0
        self.dist_to_goal: float = 0.0
        self.current_pitch_deg: float = 0.0

    def _init_planners(self) -> None:
        """初始化全局与局部规划器"""
        if self.elevation_map is None:
            return
        self.global_planner = GlobalPlannerAStar(self.elevation_map, cost_weight=3.5)
        self.local_planner = LocalPlannerDWA(
            costmap=self.elevation_map,
            max_vel_x=0.50,
            min_vel_x=0.15,
            max_vel_theta=1.8,
            acc_lim_x=1.8,
            acc_lim_theta=4.0,
            sim_time=1.5,
            sim_granularity=0.1,
            vx_samples=9,
            vtheta_samples=25,
            acc_window_dt=0.35,
            heading_cost_weight=0.55,
            obstacle_cost_weight=0.15,
            velocity_cost_weight=0.30,
            goal_tolerance=0.30,
        )

    def set_map(self, elevation_map: ElevationMap2D) -> None:
        """注入 2.5D 高程感知地图"""
        with self._lock:
            self.elevation_map = elevation_map
            self._init_planners()

    def load_map_from_storage(self, map_name_or_path: str, maps_root: str = "maps") -> bool:
        """从持久化存储中读取并装载地图"""
        try:
            elev_map, _, _ = MapStorage.load_map(map_name_or_path, maps_root=maps_root)
            self.set_map(elev_map)
            return True
        except Exception as err:
            print(f"[NavigationManager] 加载地图失败: {err}")
            return False

    def navigate_to(
        self,
        goal_x: float,
        goal_y: float,
        goal_yaw: Optional[float] = None,
        via_points: Optional[List[Tuple[float, float]]] = None,
        async_run: bool = True,
    ) -> bool:
        """
        下发导航目标并启动规划与巡航

        参数:
            goal_x: 目标点物理坐标 X
            goal_y: 目标点物理坐标 Y
            goal_yaw: 到达时的目标航向偏航角 (可选)
            via_points: 必须途经的显式安全走廊引导航点 (可选)
            async_run: 是否在后台线程异步执行 (默认 True)
        返回:
            规划与启动是否成功
        """
        with self._lock:
            if self.state in (NavigationState.PLANNING, NavigationState.NAVIGATING):
                self._stop_requested = True
            if self.elevation_map is None or self.global_planner is None:
                print("[NavigationManager] 错误: 尚未装载高程地图，无法寻路！")
                return False

            self.goal_x = float(goal_x)
            self.goal_y = float(goal_y)
            self.goal_yaw = float(goal_yaw) if goal_yaw is not None else None
            self._stop_requested = False
            self.state = NavigationState.PLANNING

            # 清空上一轮遥测缓存
            self.robot_trajectory_2d.clear()
            self.robot_trajectory_3d.clear()
            self.time_series.clear()
            self.vx_history.clear()
            self.vw_history.clear()
            self.pitch_history.clear()
            self.dist_history.clear()
            self.current_vx = 0.0
            self.current_vw = 0.0

            # 1. 计算 A* 全局拓扑路径
            start_pos = self.sim.get_robot_position()
            start_point = (float(start_pos[0]), float(start_pos[1]))
            goal_point = (self.goal_x, self.goal_y)

            path_segments: List[List[Tuple[float, float]]] = []
            current_start = start_point

            checkpoints = list(via_points) if via_points else []
            checkpoints.append(goal_point)

            for cp in checkpoints:
                seg = self.global_planner.plan(current_start, cp, smooth=True)
                if not seg:
                    print(f"[NavigationManager] A* 路径规划在分段 {current_start} -> {cp} 失败！")
                    self.state = NavigationState.BLOCKED
                    return False
                path_segments.append(seg)
                current_start = cp

            # 缝合全局连续路径
            full_path = path_segments[0]
            for s in path_segments[1:]:
                full_path += s[1:]
            self.global_path = full_path

            self.state = NavigationState.NAVIGATING

        if async_run:
            if self._worker_thread is not None and self._worker_thread.is_alive():
                self._worker_thread.join(timeout=0.5)
            self._worker_thread = threading.Thread(target=self._navigation_loop, daemon=True)
            self._worker_thread.start()
            return True
        else:
            return self._run_synchronous_loop()

    def cancel_goal(self) -> None:
        """取消当前导航任务并平稳刹车"""
        with self._lock:
            self._stop_requested = True
            self.state = NavigationState.CANCELLED
            self._brake()

    def emergency_stop(self) -> None:
        """紧急制动抱闸"""
        with self._lock:
            self._stop_requested = True
            self.state = NavigationState.EMERGENCY_STOP
            self._brake()

    def _brake(self) -> None:
        """底盘刹车执行"""
        self.current_vx = 0.0
        self.current_vw = 0.0
        self.driver.emergency_stop()
        w_left, w_right = self.driver.get_wheel_angular_velocities()
        self.sim.apply_wheel_controls(w_left, w_right)

    def _get_pitch_angle(self) -> float:
        """解算车身爬坡俯仰角 (Pitch Angle)"""
        try:
            joint_id = self.sim.model.joint("root").id
            qpos_addr = self.sim.model.jnt_qposadr[joint_id]
            qw = self.sim.data.qpos[qpos_addr + 3]
            qx = self.sim.data.qpos[qpos_addr + 4]
            qy = self.sim.data.qpos[qpos_addr + 5]
            qz = self.sim.data.qpos[qpos_addr + 6]
            sinp = 2.0 * (qw * qy - qz * qx)
            sinp = max(-1.0, min(1.0, sinp))
            return math.degrees(math.asin(sinp))
        except Exception:
            return 0.0

    def step_control_cycle(self) -> bool:
        """
        单步控制周期迭代 (周期 dt_control)
        返回 True 表示已到达终点或终止，False 表示仍在行进
        """
        if self.state != NavigationState.NAVIGATING or self.local_planner is None:
            return True

        curr_pos = self.sim.get_robot_position()
        curr_yaw = self.sim.get_robot_yaw()
        curr_x, curr_y, curr_z = float(curr_pos[0]), float(curr_pos[1]), float(curr_pos[2])
        self.current_pitch_deg = self._get_pitch_angle()

        dist_to_goal = math.hypot(self.goal_x - curr_x, self.goal_y - curr_y)
        self.dist_to_goal = dist_to_goal

        # 记录遥测历史
        self.robot_trajectory_2d.append((curr_x, curr_y))
        self.robot_trajectory_3d.append((curr_x, curr_y, curr_z))
        self.pitch_history.append(self.current_pitch_deg)
        self.dist_history.append(dist_to_goal)
        t_now = len(self.time_series) * self.dt_control
        self.time_series.append(t_now)

        # 终点抵达判定 (容差 0.30m)
        if dist_to_goal <= 0.30:
            with self._lock:
                self.state = NavigationState.ARRIVED
                self._brake()
                self.vx_history.append(0.0)
                self.vw_history.append(0.0)
            return True

        # 提取前瞻引导子目标
        subgoal = self.local_planner.get_subgoal_from_path(
            (curr_x, curr_y, curr_yaw), self.global_path, lookahead_dist=0.55
        )
        self.current_subgoal = subgoal
        is_final = (subgoal == self.global_path[-1])

        # DWA 计算速度指令
        best_vx, best_vw, _ = self.local_planner.compute_velocity_commands(
            (curr_x, curr_y, curr_yaw),
            (self.current_vx, self.current_vw),
            subgoal,
            dt=self.dt_control,
            is_final_goal=is_final,
        )

        self.current_vx = best_vx
        self.current_vw = best_vw
        self.vx_history.append(best_vx)
        self.vw_history.append(best_vw)

        # 逆运动学驱动下发
        self.driver.set_velocity(best_vx, best_vw)
        w_left, w_right = self.driver.get_wheel_angular_velocities()
        self.sim.apply_wheel_controls(w_left, w_right)

        # 步进物理引擎
        substeps = max(1, int(self.dt_control / self.sim.timestep))
        for _ in range(substeps):
            self.sim.step()

        return False

    def _navigation_loop(self) -> None:
        """后台异步控制循环"""
        max_steps = 1200
        step = 0
        while not self._stop_requested and step < max_steps:
            finished = self.step_control_cycle()
            if finished:
                break
            step += 1
            time.sleep(self.dt_control)

        if step >= max_steps and self.state == NavigationState.NAVIGATING:
            with self._lock:
                self.state = NavigationState.BLOCKED
                self._brake()

    def _run_synchronous_loop(self, max_steps: int = 1000) -> bool:
        """同步阻塞式运行控制循环"""
        for _ in range(max_steps):
            if self._stop_requested:
                break
            if self.step_control_cycle():
                return self.state == NavigationState.ARRIVED
        return self.state == NavigationState.ARRIVED

    def get_telemetry(self) -> Dict[str, Any]:
        """获取当前实时的导航状态与动力学遥测快照"""
        with self._lock:
            curr_pos = self.sim.get_robot_position()
            curr_yaw = self.sim.get_robot_yaw()
            return {
                "state": self.state.value,
                "position": [float(curr_pos[0]), float(curr_pos[1]), float(curr_pos[2])],
                "yaw_rad": float(curr_yaw),
                "yaw_deg": math.degrees(curr_yaw),
                "pitch_deg": float(self.current_pitch_deg),
                "linear_velocity": float(self.current_vx),
                "angular_velocity": float(self.current_vw),
                "goal": [self.goal_x, self.goal_y],
                "distance_to_goal": float(self.dist_to_goal),
                "subgoal": [float(self.current_subgoal[0]), float(self.current_subgoal[1])],
                "trajectory_length": len(self.robot_trajectory_2d),
                "path_waypoints_count": len(self.global_path),
            }
