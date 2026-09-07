#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
移动机器人交互仿真与 GStreamer 双机位低延迟推流统一服务
车载前向相机与高空监控相机通过 GStreamer H.264 RTP 实时广播至上位机。
  - 机位 01 (front_cam 车载驾驶感知): UDP: 5002
  - 机位 02 (overview_cam 全局监控):  UDP: 5004
"""

import argparse
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

# 优先导入平台兼容层
import simulation.platform_compat

try:
    from simulation import (
        TurtleBot3BurgerDriver,
        UrbanWorldSimulation,
        TeleopKeyboard,
        MultiCameraStreamServer,
    )
except ImportError as err:
    print(f"[Error] 模块加载失败: {err}")
    print("[Tip] 请激活虚拟环境: source venv/bin/activate")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="移动机器人 GStreamer 双机位低延迟推流与物理仿真服务")
    parser.add_argument(
        "--gui",
        action="store_true",
        default=False,
        help="是否启动 MuJoCo 原生 3D 视窗 (默认关闭，纯终端静默控制与推流)",
    )
    args = parser.parse_args()

    scene_path = os.path.join(PROJECT_ROOT, "assets", "scenes", "urban_world.xml")
    if not os.path.exists(scene_path):
        print(f"[Error] 未找到场景文件: {scene_path}")
        sys.exit(1)

    print("==================================================================")
    print("   mobile_robot_station - GStreamer 视讯推流与移动物理仿真服务    ")
    print("==================================================================")
    print(f" [视窗状态]: {'开启原生 3D 视窗' if args.gui else '无弹窗模式 (纯终端直接键盘遥控，无焦点抢占)'}")
    print(" [推流频道]:")
    print("   - 车载感知相机 (front_cam)   : rtp://127.0.0.1:5002 (H.264, 30fps)")
    print("   - 全局监控相机 (overview_cam): rtp://127.0.0.1:5004 (H.264, 30fps)")
    print(" [终端键盘遥控]:")
    print("   - W / S : 增加 / 减小前进线速度 (v)")
    print("   - A / D : 增加左转 / 右转角速度 (w)")
    print("   - 空格键: 紧急制动刹车 (Emergency Stop)")
    print("   - Q/ESC : 安全退出")
    print("==================================================================\n")

    sim = UrbanWorldSimulation(scene_path)
    driver = TurtleBot3BurgerDriver()

    # 初始化双机位推流服务
    stream_server = MultiCameraStreamServer(
        width=640,
        height=480,
        fps=30,
        host="127.0.0.1",
        front_port=5002,
        overview_port=5004,
    )

    last_stream_time = 0.0
    stream_interval = 1.0 / 30.0  # 30 FPS
    last_hud_time = 0.0

    viewer = None
    if args.gui:
        try:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
            viewer.cam.distance = 5.0
            viewer.cam.elevation = -35
            viewer.cam.azimuth = -135
            viewer.cam.lookat = np.array([0.0, -5.0, 0.5])
        except Exception as e:
            print(f"[Warning] 无法打开视窗 ({e})，降级为无弹窗模式。")
            viewer = None

    with TeleopKeyboard() as keyboard:
        try:
            while True:
                step_start = time.time()

                # 键盘输入 (终端直接捕获，无任何窗口抢占焦点)
                key = keyboard.poll_key()
                if key in ['q', 'Q', '\x1b']:
                    print("\n[Info] 正在退出仿真推流服务...")
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

                # 动力学步进
                w_l, w_r = driver.get_wheel_angular_velocities()
                sim.apply_wheel_controls(w_l, w_r)
                sim.step()

                # 30FPS 定时捕获画面并推流
                cur_time = time.time()
                if cur_time - last_stream_time >= stream_interval:
                    last_stream_time = cur_time
                    # 离屏渲染双机位画面并推入 GStreamer 管道
                    frame_front = sim.render_camera("front_cam")
                    frame_overview = sim.render_camera("overview_cam")
                    stream_server.push_front_frame(frame_front)
                    stream_server.push_overview_frame(frame_overview)

                # 若开启视窗，则同步视口相机跟随
                if viewer is not None:
                    if not viewer.is_running():
                        print("\n[Info] 视窗已关闭，退出服务...")
                        break
                    car_pos = sim.get_robot_position()
                    viewer.cam.lookat = car_pos
                    viewer.sync()

                # 5Hz 终端遥测刷新
                if cur_time - last_hud_time >= 0.2:
                    last_hud_time = cur_time
                    car_pos = sim.get_robot_position()
                    yaw = sim.get_robot_yaw()
                    hud_str = (
                        f"\r[TERMINAL-CTRL] Pos: ({car_pos[0]:5.2f}, {car_pos[1]:5.2f}, {car_pos[2]:5.2f})m | "
                        f"Yaw: {math.degrees(yaw):5.1f}° | "
                        f"Cmd: v={driver.v:4.2f}m/s, w={driver.w:4.2f}rad/s | "
                        f"Streaming: 5002 & 5004 (30fps)"
                    )
                    sys.stdout.write(hud_str)
                    sys.stdout.flush()

                # 步进节拍同步
                remain = sim.timestep - (time.time() - step_start)
                if remain > 0:
                    time.sleep(remain)

        except KeyboardInterrupt:
            print("\n[Info] 捕获中断信号 (Ctrl+C)，正在安全退出...")
        finally:
            if viewer is not None:
                try:
                    viewer.close()
                except Exception:
                    pass
            stream_server.close()
            print("\n[Info] GStreamer 双机位推流服务已安全关闭并释放端口。")


if __name__ == "__main__":
    main()

