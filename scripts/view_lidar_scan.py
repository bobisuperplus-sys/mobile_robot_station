#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D 激光雷达点云实时扫描与交互式遥控演示脚本
小车在立体建筑群中穿梭行驶，车载 16 线激光雷达实时向四周发射光线，实时捕获周围建筑物与道路点云。
"""

import sys
import os
import time
import math
import numpy as np

# 导入工程根路径
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

# 优先导入平台兼容层，强制适配 X11/XWayland 后端并静音 Wayland 窗口位置警告
import src.simulation.platform_compat

try:
    import mujoco.viewer
    from src.simulation import (
        TurtleBot3BurgerDriver,
        UrbanWorldSimulation,
        TeleopKeyboard,
        RaycastLidar3D,
    )
except ImportError as err:
    print(f"[Error] 模块加载失败: {err}")
    print("[Tip] 请激活虚拟环境: source /home/yellowtown/Code/PythonProject/venv/bin/activate")
    sys.exit(1)


def main():
    scene_path = os.path.join(PROJECT_ROOT, "assets", "scenes", "urban_world.xml")
    if not os.path.exists(scene_path):
        print(f"[Error] 未找到场景文件: {scene_path}")
        sys.exit(1)

    print("==================================================================")
    print("      mobile_robot_station - 3D 激光雷达点云实时扫描系统          ")
    print("==================================================================")
    print(" [控制说明]:")
    print("   - W / S : 增加 / 减小前进线速度 (v)")
    print("   - A / D : 增加左转 / 右转角速度 (w)")
    print("   - 空格键: 紧急制动刹车 (Emergency Stop)")
    print("   - P 键  : 保存当前帧 3D 点云至 PCD/XYZ 文件")
    print("   - Q/ESC : 退出程序")
    print("==================================================================\n")

    # 1. 实例化 16 线 3D 激光雷达与物理环境
    lidar = RaycastLidar3D(
        vertical_channels=16,
        vertical_fov_deg=(-15.0, 15.0),
        horizontal_resolution_deg=3.0,
        min_range=0.15,
        max_range=25.0,
        noise_std=0.01,
    )
    sim = UrbanWorldSimulation(scene_path, lidar=lidar)
    driver = TurtleBot3BurgerDriver()

    last_scan_time = 0.0
    scan_interval = 0.10  # 10Hz 标称扫描频率
    last_hud_time = 0.0
    latest_scan = None

    with TeleopKeyboard() as keyboard:
        try:
            with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
                viewer.cam.distance = 5.0
                viewer.cam.elevation = -35
                viewer.cam.azimuth = -135
                viewer.cam.lookat = np.array([0.0, -5.0, 0.5])

                while viewer.is_running():
                    step_start = time.time()

                    # 键盘输入检测
                    key = keyboard.poll_key()
                    if key in ['q', 'Q', '\x1b']:
                        print("\n[Info] 正在退出仿真会话...")
                        break
                    elif key in ['w', 'W']:
                        driver.step_forward()
                    elif key in ['s', 'S']:
                        driver.step_backward()
                    elif key in ['a', 'A']:
                        driver.step_turn_left()
                    elif key in ['d', 'D']:
                        driver.step_turn_right()
                    elif key in [' ', 'x', 'X']:
                        driver.emergency_stop()
                    elif key in ['p', 'P']:
                        if latest_scan is not None and len(latest_scan["points"]) > 0:
                            save_dir = os.path.join(PROJECT_ROOT, "tests", "data")
                            os.makedirs(save_dir, exist_ok=True)
                            save_file = os.path.join(save_dir, f"scan_{int(time.time())}.xyz")
                            np.savetxt(save_file, latest_scan["points"], fmt="%.4f")
                            print(f"\n[Info] 当前帧 3D 点云 ({len(latest_scan['points'])} 点) 已导出至: {save_file}")

                    # 下发动力学控制
                    w_left, w_right = driver.get_wheel_angular_velocities()
                    sim.apply_wheel_controls(w_left, w_right)

                    # 物理步进
                    sim.step()

                    # 相机跟随
                    car_pos = sim.get_robot_position()
                    viewer.cam.lookat = car_pos

                    # 10Hz 定时触发 3D 激光雷达扫描
                    cur_time = time.time()
                    if cur_time - last_scan_time >= scan_interval:
                        scan_t0 = time.perf_counter()
                        latest_scan = sim.get_lidar_pointcloud(return_world_frame=False)
                        scan_dt = (time.perf_counter() - scan_t0) * 1000.0
                        last_scan_time = cur_time

                    # 5Hz 刷新终端遥测与点云诊断
                    if cur_time - last_hud_time >= 0.2:
                        last_hud_time = cur_time
                        yaw = sim.get_robot_yaw()
                        pts_count = latest_scan["valid_count"] if latest_scan else 0
                        min_dist = np.min(latest_scan["ranges"]) if (latest_scan and pts_count > 0) else 0.0

                        hud_str = (
                            f"\r[LIDAR-TELEMETRY] Pos: ({car_pos[0]:5.2f}, {car_pos[1]:5.2f}, {car_pos[2]:5.2f})m | "
                            f"Yaw: {math.degrees(yaw):5.1f}° | "
                            f"Lidar: {pts_count:4d}/{lidar.total_rays} pts | "
                            f"Closest: {min_dist:4.2f}m | "
                            f"Scan Latency: {scan_dt:4.1f}ms"
                        )
                        sys.stdout.write(hud_str)
                        sys.stdout.flush()

                    viewer.sync()

                    # 节拍同步
                    remain = sim.timestep - (time.time() - step_start)
                    if remain > 0:
                        time.sleep(remain)

        finally:
            print("\n[Info] 激光雷达实时扫描会话已安全关闭。")


if __name__ == "__main__":
    main()
