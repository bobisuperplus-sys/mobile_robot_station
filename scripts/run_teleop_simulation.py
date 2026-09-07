#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
移动机器人交互式键盘遥控与物理仿真启动入口
环境依赖: /home/yellowtown/Code/PythonProject/venv
"""

import sys
import os
import time
import math
import numpy as np

# 将工程根目录追加至模块检索路径
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

try:
    import mujoco.viewer
    from src.simulation import (
        TurtleBot3BurgerDriver,
        UrbanWorldSimulation,
        TeleopKeyboard,
    )
except ImportError as err:
    print(f"[Error] 模块加载失败: {err}")
    print("[Tip] 请确保已激活指定虚拟环境: source /home/yellowtown/Code/PythonProject/venv/bin/activate")
    sys.exit(1)


def print_banner():
    """输出系统控制面板提示信息 (纯文本，严禁 Emoji)"""
    print("==================================================================")
    print("      mobile_robot_station - 移动机器人 3D 建筑群物理仿真系统      ")
    print("==================================================================")
    print(" [控制说明]:")
    print("   - W / S : 增加 / 减小前进线速度 (v, 最大 0.22 m/s)")
    print("   - A / D : 增加左转 / 右转角速度 (w, 最大 2.84 rad/s)")
    print("   - 空格键: 紧急制动刹车 (Emergency Stop)")
    print("   - Q/ESC : 退出仿真程序")
    print("==================================================================\n")


def main():
    scene_path = os.path.join(PROJECT_ROOT, "assets", "scenes", "urban_world.xml")
    if not os.path.exists(scene_path):
        print(f"[Error] 未找到场景文件: {scene_path}")
        sys.exit(1)

    print_banner()

    # 1. 实例化仿真世界管理器与底盘运动学驱动器
    sim = UrbanWorldSimulation(scene_path)
    driver = TurtleBot3BurgerDriver()

    last_hud_time = 0.0

    # 2. 启动终端非阻塞按键监听与 MuJoCo 轻量视口
    with TeleopKeyboard() as keyboard:
        try:
            with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
                # 配置初始视点
                viewer.cam.distance = 4.5
                viewer.cam.elevation = -30
                viewer.cam.azimuth = -135
                viewer.cam.lookat = np.array([0.0, -5.0, 0.5])

                while viewer.is_running():
                    step_start = time.time()

                    # 捕获键盘事件
                    key = keyboard.poll_key()
                    if key in ['q', 'Q', '\x1b']:
                        print("\n[Info] 收到退出指令，正在终止仿真...")
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

                    # 差速运动学逆解轮速
                    w_left, w_right = driver.get_wheel_angular_velocities()
                    sim.apply_wheel_controls(w_left, w_right)

                    # 物理步进
                    sim.step()

                    # 镜头跟随小车位置
                    car_pos = sim.get_robot_position()
                    viewer.cam.lookat = car_pos
                    viewer.sync()

                    # 5Hz 终端遥测刷新
                    cur_time = time.time()
                    if cur_time - last_hud_time > 0.2:
                        last_hud_time = cur_time
                        yaw = sim.get_robot_yaw()
                        imu_data = sim.get_imu_telemetry()
                        gyro = imu_data["angular_velocity"]

                        hud_str = (
                            f"\r[TELEMETRY] Pos: ({car_pos[0]:6.2f}, {car_pos[1]:6.2f}, {car_pos[2]:6.2f}) m | "
                            f"Yaw: {math.degrees(yaw):6.1f} deg | "
                            f"Cmd: v={driver.v:4.2f} m/s, w={driver.w:4.2f} rad/s | "
                            f"Gyro-Z: {gyro[2]:5.2f} rad/s"
                        )
                        sys.stdout.write(hud_str)
                        sys.stdout.flush()

                    # 同步物理节拍
                    elapsed = time.time() - step_start
                    remain = sim.timestep - elapsed
                    if remain > 0:
                        time.sleep(remain)

        finally:
            print("\n[Info] 物理仿真会话已安全关闭。")


if __name__ == "__main__":
    main()
