#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MSH (Mobile Station Host) 主控微服务总线与数字孪生后台启动控制台

核心职责：
1. 启动并托管 MuJoCo 物理动力学世界与差速底盘驱动器；
2. 启动并托管 2.5D 高程自主导航协调管理器 (NavigationManager) 与地图存储 (MapStorage)；
3. 启动基于 TCP JSON-RPC 2.0 (默认 9001 端口) 的服务网关 MSHServer，对外发布全车能力清单；
4. 默认拉起 MuJoCo 原生 3D 渲染窗口跟随小车，支持上位机 (Qt 控制台) 与终端客户端远程调度作业。
"""

import argparse
import os
import sys
import threading
import time
import numpy as np

# 动态相对寻址添加工程根目录，杜绝绝对路径硬编码
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

import simulation.platform_compat
from msh import MSHServer, ServiceHandler, get_robot_capabilities
from msh.client import print_formatted_capabilities
from navigation import ElevationMap2D, MapStorage, NavigationManager, NavigationState
from simulation import TurtleBot3BurgerDriver, UrbanWorldSimulation, MultiCameraStreamServer


def collect_initial_elevation_map(sim: UrbanWorldSimulation) -> ElevationMap2D:
    """初始化构建场景 2.5D 高程与坡度地图"""
    import math
    import mujoco

    joint_id = sim.model.joint("root").id
    qpos_addr = sim.model.jnt_qposadr[joint_id]
    qvel_addr = sim.model.jnt_dofadr[joint_id]

    survey_poses = [
        (0.0, -10.0, 0.01, math.pi / 2.0),
        (0.0, -6.0, 0.01, math.pi / 2.0),
        (-3.2, 0.0, 0.01, math.pi / 2.0),
        (-2.5, -2.5, 0.01, math.pi / 4.0),
        (2.5, -2.5, 0.01, 3.0 * math.pi / 4.0),
        (-1.5, 2.5, 0.01, math.pi / 4.0),
        (0.0, 2.4, 0.01, math.pi / 2.0),
        (0.0, 4.0, 0.05, math.pi / 2.0),
        (0.0, 6.0, 0.15, math.pi / 2.0),
    ]

    scans = []
    for x, y, z, yaw in survey_poses:
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        sim.data.qpos[qpos_addr : qpos_addr + 7] = [x, y, z, qw, 0.0, 0.0, qz]
        sim.data.qvel[qvel_addr : qvel_addr + 6] = 0.0
        mujoco.mj_forward(sim.model, sim.data)
        scan = sim.get_lidar_pointcloud(return_world_frame=True)
        scans.append(scan["points"])

    # 复位至出发点
    sim.data.qpos[qpos_addr : qpos_addr + 7] = [0.0, -6.0, 0.01, math.cos(math.pi / 4.0), 0.0, 0.0, math.sin(math.pi / 4.0)]
    sim.data.qvel[qvel_addr : qvel_addr + 6] = 0.0
    mujoco.mj_forward(sim.model, sim.data)
    for _ in range(15):
        sim.step()

    pts = np.vstack(scans)
    elev_map = ElevationMap2D(resolution=0.05, size_x=28.0, size_y=28.0, origin_x=-14.0, origin_y=-14.0)
    elev_map.update_from_point_cloud(pts)

    # 自动存盘为默认底图
    maps_root = os.path.join(PROJECT_ROOT, "maps")
    MapStorage.save_map("urban_world_default", elev_map, point_cloud=pts, output_dir=maps_root, description="场景基准 2.5D 高程底图")
    return elev_map


def main():
    parser = argparse.ArgumentParser(description="MSH 移动机器人主控微服务总线控制台")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="RPC 监听 IP (默认 0.0.0.0)")
    parser.add_argument("--port", type=int, default=9001, help="RPC 监听端口 (默认 9001)")
    parser.add_argument("--headless", action="store_true", default=False, help="启用无头模式 (默认自动拉起 3D 视口)")
    parser.add_argument("--stream", action=argparse.BooleanOptionalAction, default=True, help="是否启用 GStreamer 双机位低延迟推流 (默认开启，UDP 5002/5004)")
    args = parser.parse_args()

    scene_xml = os.path.join(PROJECT_ROOT, "assets", "scenes", "urban_world.xml")
    if not os.path.exists(scene_xml):
        print(f"[Error] 场景文件不存在: {scene_xml}")
        sys.exit(1)

    print("======================================================================")
    print("  Mobile Station Host (MSH) - 主控微服务总线与数字孪生系统")
    print("======================================================================")

    # 1. 打印全车能力清单
    cap = get_robot_capabilities()
    print_formatted_capabilities(cap)

    # 2. 实例化物理仿真与底盘
    print("\n[1/4] 初始化 MuJoCo 物理动力学引擎与传感器底座...")
    sim = UrbanWorldSimulation(scene_xml)
    driver = TurtleBot3BurgerDriver(wheel_radius=0.033, wheel_base=0.160, max_v=0.60, max_w=3.5)

    # 3. 初始化 2.5D 高程与自主导航管理器
    print("[2/4] 初始化 2.5D 高程感知、地图持久化与自主导航协调器...")
    elev_map = collect_initial_elevation_map(sim)
    nav_mgr = NavigationManager(sim=sim, driver=driver, elevation_map=elev_map)

    # 4. 启动 MSH JSON-RPC 2.0 服务网关
    print(f"[3/4] 启动 MSH JSON-RPC 2.0 服务网关 (监听 {args.host}:{args.port})...")
    maps_root = os.path.join(PROJECT_ROOT, "maps")
    service_handler = ServiceHandler(sim=sim, driver=driver, navigation_mgr=nav_mgr, maps_root=maps_root)
    msh_server = MSHServer(service_handler=service_handler, host=args.host, port=args.port)
    msh_server.start(blocking=False)

    # 5. 启动 GStreamer 双机位低延迟视讯推流 (与仿真动力学完全绑定同一真值源)
    stream_server = None
    if args.stream:
        print(f"[4/4] 启动 GStreamer 双机位低延迟视讯推流服务...")
        try:
            stream_server = MultiCameraStreamServer(
                width=640,
                height=480,
                fps=30,
                host="127.0.0.1",
                front_port=5002,
                overview_port=5004,
            )
            print("[OK] GStreamer 双机位推流已就绪！")
            print("     - 机位 01 (车载前视第一人称): udp://127.0.0.1:5002 (H.264, 30fps)")
            print("     - 机位 02 (全局高空俯视监控): udp://127.0.0.1:5004 (H.264, 30fps)")
        except Exception as err:
            print(f"[Warn] 无法拉起 GStreamer 推流: {err}")
            stream_server = None
    else:
        print("[4/4] GStreamer 视讯推流已被用户参数禁用 (--no-stream)。")

    print("\n======================================================================")
    print(f"[OK] MSH 主控微服务已一体化就绪！")
    print(f"     - RPC 通信地址: tcp://{args.host}:{args.port}")
    print(f"     - 查看能力清单: python -m msh.client --capabilities")
    print(f"     - 下发导航指令: python -m msh.client --nav 0.0 6.0")
    print(f"     - 遥控小车移动: python -m msh.client --drive 0.3 0.1")
    print("======================================================================")

    viewer = None
    if not args.headless:
        try:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
            viewer.cam.distance = 7.0
            viewer.cam.elevation = -28
            viewer.cam.azimuth = -135
            print("[OK] 已拉起原生 3D 渲染窗口，实时展示小车姿态与外界指令响应...")
        except Exception as err:
            print(f"[Warn] 无法拉起 3D 视口: {err}")
            viewer = None

    stream_thread_running = True

    def stream_worker():
        nonlocal stream_thread_running
        last_time = 0.0
        interval = 1.0 / 30.0  # 30 FPS
        while stream_thread_running:
            now = time.time()
            if now - last_time < interval:
                time.sleep(max(0.001, interval - (now - last_time)))
                continue
            last_time = now
            if stream_server is not None:
                try:
                    frame_front = sim.render_camera("front_cam")
                    cam_name = "third_person_cam" if "third_person_cam" in [sim.model.cam(i).name for i in range(sim.model.ncam)] else "overview_cam"
                    frame_overview = sim.render_camera(cam_name)
                    stream_server.push_front_frame(frame_front)
                    stream_server.push_overview_frame(frame_overview)
                except Exception:
                    pass

    if stream_server is not None:
        t_stream = threading.Thread(target=stream_worker, daemon=True)
        t_stream.start()

    try:
        while True:
            step_start = time.time()

            if viewer is not None and not viewer.is_running():
                print("\n[Info] 3D 视口已关闭，安全退出 MSH...")
                break

            # 遥控速度看门狗检测 (超时 0.6s 无新指令自动归零，杜绝无限自转)
            service_handler.check_watchdog(timeout=0.6)

            # 若当前没有运行导航任务，进行物理背景步进并确保静止时驻车锁死
            is_navigating = (nav_mgr.state == NavigationState.NAVIGATING)
            if not is_navigating:
                is_stopped = (driver.v == 0.0 and driver.w == 0.0) or (
                    nav_mgr.state in (
                        NavigationState.ARRIVED,
                        NavigationState.IDLE,
                        NavigationState.BLOCKED,
                        NavigationState.CANCELLED,
                        NavigationState.EMERGENCY_STOP,
                    )
                )
                if is_stopped:
                    driver.v = 0.0
                    driver.w = 0.0
                    sim.apply_wheel_controls(0.0, 0.0)
                    with sim._lock:
                        sim.data.qvel[:] = 0.0
                sim.step()

            if viewer is not None and viewer.is_running():
                pos = sim.get_robot_position()
                viewer.cam.lookat = np.array([float(pos[0]), float(pos[1]), max(0.3, float(pos[2]) + 0.1)])
                viewer.sync()

            remain = sim.timestep - (time.time() - step_start)
            if remain > 0:
                time.sleep(remain)
    except KeyboardInterrupt:
        print("\n[Info] 收到中断信号，正在关闭服务...")
    finally:
        stream_thread_running = False
        msh_server.stop()
        if stream_server is not None:
            stream_server.close()
        if viewer is not None and viewer.is_running():
            viewer.close()
        print("[OK] MSH 主控服务与视讯推流已完全关闭。")


if __name__ == "__main__":
    main()
