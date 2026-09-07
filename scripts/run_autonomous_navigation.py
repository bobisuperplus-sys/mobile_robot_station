#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
移动机器人 2.5D 高程感知、立体越障爬坡与自主导航工作站控制台

核心特性：
1. 2.5D 地形高程与坡度图 (Elevation & Slope Map): 3D 激光点云高程解算、二维空间梯度坡度估计与局部台阶高差检测；
2. 三维立体地形可通行性评估 (Traversability): 平坦道路低代价通行，缓坡平滑放行引导爬坡，垂直硬台阶阻断避让；
3. A* 全局路径规划与 DWA 局部动态避障全闭环，支持自主穿行街道、爬上坡道并在高台精准驻车制动；
4. 默认启动 MuJoCo 原生 3D 渲染窗口实时跟随小车，生成集成 2.5D 高程图、3D 立体地形流形、双机位视讯与动力学遥测的超高清态势大图。
"""

import argparse
import math
import os
import shutil
import sys
import time
from typing import List, Tuple

import matplotlib
matplotlib.use("Agg")  # 强制无头离屏渲染后端
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 显式激活 3D 投影支持
from matplotlib import font_manager
import numpy as np

# 动态相对寻址添加工程根目录，杜绝绝对路径硬编码
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

# 优先导入跨平台兼容层
import simulation.platform_compat

from simulation import (
    TurtleBot3BurgerDriver,
    UrbanWorldSimulation,
)
from navigation import (
    Costmap2D,
    ElevationMap2D,
    GlobalPlannerAStar,
    LocalPlannerDWA,
)


def setup_chinese_font():
    """配置 Matplotlib 中文字体支持，优先选用系统内置 Noto Sans CJK 或文泉驿"""
    candidates = [
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc",
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    ]
    for font_path in candidates:
        if os.path.exists(font_path):
            font_prop = font_manager.FontProperties(fname=font_path)
            font_manager.fontManager.addfont(font_path)
            plt.rcParams["font.family"] = font_prop.get_name()
            plt.rcParams["axes.unicode_minus"] = False
            return font_prop
    return None


def load_pointcloud_ply(file_path: str) -> np.ndarray:
    """读取标准 ASCII PLY 格式点云文件"""
    points = []
    with open(file_path, "r", encoding="utf-8") as f:
        in_header = True
        for line in f:
            line = line.strip()
            if in_header:
                if line == "end_header":
                    in_header = False
                continue
            if line:
                parts = line.split()
                if len(parts) >= 3:
                    points.append([float(parts[0]), float(parts[1]), float(parts[2])])
    return np.array(points, dtype=np.float32)


def collect_scene_pointcloud(sim: UrbanWorldSimulation) -> np.ndarray:
    """在场景关键视点快速触发激光雷达全向感知，构建高保真立体环境点云 (覆盖建筑、中央环岛与高台坡道)"""
    import mujoco
    survey_poses = [
        (0.0, -6.0, 0.01, math.pi / 2.0),
        (-3.2, -3.5, 0.01, math.pi / 3.0),
        (-3.2, 0.0, 0.01, math.pi / 2.0),
        (-3.2, 2.6, 0.01, math.pi / 4.0),
        (0.0, 3.0, 0.01, math.pi / 2.0),
        (0.0, 6.0, 0.15, math.pi / 2.0),
    ]
    joint_id = sim.model.joint("root").id
    qpos_addr = sim.model.jnt_qposadr[joint_id]
    qvel_addr = sim.model.jnt_dofadr[joint_id]

    scans = []
    for x, y, z, yaw in survey_poses:
        qw = math.cos(yaw / 2.0)
        qz = math.sin(yaw / 2.0)
        sim.data.qpos[qpos_addr : qpos_addr + 7] = [x, y, z, qw, 0.0, 0.0, qz]
        sim.data.qvel[qvel_addr : qvel_addr + 6] = 0.0
        mujoco.mj_forward(sim.model, sim.data)
        scan = sim.get_lidar_pointcloud(return_world_frame=True)
        scans.append(scan["points"])

    # 扫描完毕，将机器人精准平稳复位至南侧出发起点 (0.0, -6.0) 并预热物理步
    sim.data.qpos[qpos_addr : qpos_addr + 7] = [0.0, -6.0, 0.01, math.cos(math.pi / 4.0), 0.0, 0.0, math.sin(math.pi / 4.0)]
    sim.data.qvel[qvel_addr : qvel_addr + 6] = 0.0
    mujoco.mj_forward(sim.model, sim.data)
    for _ in range(25):
        sim.step()

    pts = np.vstack(scans)
    return pts


def run_navigation(
    goal_x: float = 0.0,
    goal_y: float = 6.0,
    headless: bool = False,
    max_steps: int = 900,
    output_image: str = "output/autonomous_navigation_dashboard.png",
):
    """执行端到端 2.5D 高程感知、立体越障爬坡与自主导航闭环仿真"""
    scene_xml = os.path.join(PROJECT_ROOT, "assets", "scenes", "urban_world.xml")
    if not os.path.exists(scene_xml):
        print(f"[Error] 场景文件不存在: {scene_xml}")
        sys.exit(1)

    print("======================================================================")
    print("  Mobile Robot Station - 2.5D 高程感知、立体坡道爬坡与自主导航控制台")
    print("======================================================================")
    print(f"[*] 场景路径: {scene_xml}")
    print(f"[*] 目标航点: ({goal_x:.2f}, {goal_y:.2f}) [北侧立体观景高台]")

    # 1. 实例化物理仿真世界与底盘逆运动学驱动器 (支持 0.60 m/s 高速物理输出)
    sim = UrbanWorldSimulation(scene_xml)
    driver = TurtleBot3BurgerDriver(wheel_radius=0.033, wheel_base=0.160, max_v=0.60, max_w=3.5)

    # 预推进物理引擎，确保接触动力学与传感器稳定
    for _ in range(25):
        sim.step()

    # 2. 采集环境 3D 全景空间点云
    print("[1/5] 采集场景 3D 激光雷达高保真空间点云...")
    t0 = time.perf_counter()
    raw_points = collect_scene_pointcloud(sim)
    t_lidar = (time.perf_counter() - t0) * 1000.0
    print(f"      点云采集完成: 点数 {len(raw_points)}, 耗时: {t_lidar:.2f} ms")

    # 3. 核心：构建 2.5D 高程图 (Elevation Map) 与三维地形可通行性评估
    print("[2/5] 构建 2.5D 高程图与三维地形可通行性评估 (Elevation Map & Traversability)...")
    t0 = time.perf_counter()
    elevation_map = ElevationMap2D(
        resolution=0.05,
        size_x=24.0,
        size_y=24.0,
        origin_x=-12.0,
        origin_y=-12.0,
        robot_radius=0.12,
        inflation_radius=0.26,
        decay_factor=5.0,
        max_slope_deg=20.0,
        max_step_height=0.10,
    )
    elevation_map.update_from_point_cloud(raw_points)
    t_elev = (time.perf_counter() - t0) * 1000.0
    print(f"      2.5D 高程图构建完成: 栅格尺寸 ({elevation_map.nx}x{elevation_map.ny}), 耗时: {t_elev:.2f} ms")

    # 4. A* 全局路径规划 (经由西侧景观大道安全走廊，对准南侧缓坡入口，爬升高台)
    start_pos = sim.get_robot_position()
    start_point = (float(start_pos[0]), float(start_pos[1]))
    goal_point = (float(goal_x), float(goal_y))
    via_avenue = (-3.2, 0.0)      # 西侧主景观大道中央
    via_approach = (-1.5, 2.5)    # 切入北侧宽阔大道 (避开中央环岛)
    via_ramp_foot = (0.0, 3.0)    # 坡道起点南端 (从 Y=3.0 零台阶平滑攀爬至高台 Y=5.0~6.0)

    print(f"[3/5] 执行 A* 全局立体寻路: 起点 {start_point} -> 大道 {via_avenue} -> 坡道口 {via_ramp_foot} -> 高台 {goal_point}...")
    t0 = time.perf_counter()
    global_planner = GlobalPlannerAStar(elevation_map, cost_weight=3.5)
    p1 = global_planner.plan(start_point, via_avenue, smooth=True)
    p2 = global_planner.plan(via_avenue, via_approach, smooth=True)
    p3 = global_planner.plan(via_approach, via_ramp_foot, smooth=True)
    p4 = global_planner.plan(via_ramp_foot, goal_point, smooth=True)
    if not (p1 and p2 and p3 and p4):
        print("[Error] A* 全局立体规划失败，未能连接全部路径段！")
        sys.exit(1)
    global_path = p1 + p2[1:] + p3[1:] + p4[1:]
    t_astar = (time.perf_counter() - t0) * 1000.0
    print(f"      A* 规划成功: 密集引导航点数 {len(global_path)}, 耗时: {t_astar:.2f} ms")

    # 5. DWA 局部避障与坡道循迹规划器初始化
    dwa_planner = LocalPlannerDWA(
        costmap=elevation_map,
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

    # 6. 启动 3D 原生可视化视口与导航闭环控制循环
    viewer = None
    if not headless:
        try:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
            viewer.cam.distance = 7.0
            viewer.cam.elevation = -28
            viewer.cam.azimuth = -135
            viewer.cam.lookat = np.array([start_point[0], start_point[1], 0.3])
            print("[OK] 已成功拉起原生 3D 物理渲染窗口，实时镜头跟随小车自主巡航与爬坡...")
        except Exception as err:
            print(f"[Warn] 无法拉起原生 3D 视口窗口 (可能缺少显示环境): {err}")
            viewer = None

    print("[4/5] 启动差速底盘 DWA 跟踪与动态爬坡巡航...")
    robot_trajectory_2d: List[Tuple[float, float]] = []
    robot_trajectory_3d: List[Tuple[float, float, float]] = []
    time_series: List[float] = []
    vx_history: List[float] = []
    vw_history: List[float] = []
    pitch_history: List[float] = []
    dist_history: List[float] = []

    current_vx = 0.0
    current_vw = 0.0
    dt_control = 0.1
    substeps = int(dt_control / sim.timestep)

    last_traj_candidates: List[Tuple[float, float]] = []
    reached_goal = False

    joint_id = sim.model.joint("root").id
    qpos_addr = sim.model.jnt_qposadr[joint_id]

    for step in range(max_steps):
        # 视口被用户手动关闭时安全跳出
        if viewer is not None and not viewer.is_running():
            print("\n[Info] 3D 视口窗口已关闭，提前结束巡航...")
            break

        curr_pos = sim.get_robot_position()
        curr_yaw = sim.get_robot_yaw()
        curr_x, curr_y, curr_z = float(curr_pos[0]), float(curr_pos[1]), float(curr_pos[2])

        # 获取四元数并解算车体俯仰角 (Pitch Angle)
        qw = sim.data.qpos[qpos_addr + 3]
        qx = sim.data.qpos[qpos_addr + 4]
        qy = sim.data.qpos[qpos_addr + 5]
        qz = sim.data.qpos[qpos_addr + 6]
        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))
        curr_pitch_deg = math.degrees(math.asin(sinp))

        robot_trajectory_2d.append((curr_x, curr_y))
        robot_trajectory_3d.append((curr_x, curr_y, curr_z))
        pitch_history.append(curr_pitch_deg)

        dist_to_goal = math.hypot(goal_x - curr_x, goal_y - curr_y)
        dist_history.append(dist_to_goal)
        time_series.append(step * dt_control)

        # 检查是否抵达最终目标 (高台中心)
        if dist_to_goal <= 0.30:
            print(f"[Success] 小车在控制周期 #{step} 成功登上高台并抵达终点！最终位姿误差: {dist_to_goal*100:.1f} cm, 高程 Z: {curr_z:.2f} m")
            reached_goal = True
            sim.apply_wheel_controls(0.0, 0.0)
            for _ in range(substeps):
                sim.step()
            if viewer is not None and viewer.is_running():
                viewer.cam.lookat = np.array([curr_x, curr_y, curr_z])
                viewer.sync()
                time.sleep(1.5)  # 终点驻车停留
            vx_history.append(0.0)
            vw_history.append(0.0)
            break

        # 提取前瞻子目标
        subgoal = dwa_planner.get_subgoal_from_path(
            (curr_x, curr_y, curr_yaw), global_path, lookahead_dist=0.55
        )
        is_final = (subgoal == global_path[-1])

        # DWA 计算速度指令
        best_vx, best_vw, traj_rollout = dwa_planner.compute_velocity_commands(
            (curr_x, curr_y, curr_yaw),
            (current_vx, current_vw),
            subgoal,
            dt=dt_control,
            is_final_goal=is_final,
        )
        last_traj_candidates = traj_rollout

        current_vx = best_vx
        current_vw = best_vw
        vx_history.append(current_vx)
        vw_history.append(current_vw)

        # 逆运动学驱动下发
        driver.set_velocity(best_vx, best_vw)
        w_left, w_right = driver.get_wheel_angular_velocities()
        sim.apply_wheel_controls(w_left, w_right)

        # 物理步进
        for _ in range(substeps):
            sim.step()

        # 3D 视口实时镜头跟随与画面同步刷新
        if viewer is not None and viewer.is_running():
            viewer.cam.lookat = np.array([curr_x, curr_y, max(0.3, curr_z + 0.1)])
            viewer.sync()
            time.sleep(0.01)

        if step % 50 == 0:
            print(
                f"  [Step {step:04d}] 位置: ({curr_x:.2f}, {curr_y:.2f}, z={curr_z:.2f}) | "
                f"俯仰角: {curr_pitch_deg:+5.1f}° | "
                f"指令: v={best_vx:.2f} m/s, w={best_vw:.2f} rad/s | "
                f"距目标: {dist_to_goal:.2f} m"
            )

    if viewer is not None and viewer.is_running():
        viewer.close()

    # 7. 截取车载双视讯画面
    front_frame = sim.render_camera(camera_name="front_cam", width=640, height=480)
    overview_frame = sim.render_camera(camera_name="overview_cam", width=640, height=480)

    # 8. 渲染生成融合 2.5D 高程、3D 立体爬坡、双机位视讯与遥测的 2x2 旗舰级态势大图
    print(f"[5/5] 渲染生成 2.5D 高程与立体爬坡态势大图: {output_image}...")
    setup_chinese_font()

    fig = plt.figure(figsize=(20, 12), dpi=160)
    fig.patch.set_facecolor("#0b0e14")

    # ---------------- 子图 1 (左上): 2.5D 地形高程热力图与 A* / DWA 爬坡寻路 ----------------
    ax_2d = fig.add_subplot(2, 2, 1)
    ax_2d.set_facecolor("#06080c")
    extent = [
        elevation_map.origin_x,
        elevation_map.origin_x + elevation_map.size_x,
        elevation_map.origin_y,
        elevation_map.origin_y + elevation_map.size_y,
    ]
    im_elev = ax_2d.imshow(
        elevation_map.elevation_array,
        origin="lower",
        extent=extent,
        cmap="terrain",
        vmin=0.0,
        vmax=0.45,
        alpha=0.90,
    )
    cbar = plt.colorbar(im_elev, ax=ax_2d, fraction=0.038, pad=0.03)
    cbar.set_label("地表高程 Z (米)", color="#b0b8c4", fontsize=10)
    cbar.ax.tick_params(colors="#8892a0")

    # A* 全局路径
    gx = [p[0] for p in global_path]
    gy = [p[1] for p in global_path]
    ax_2d.plot(gx, gy, color="#00ffff", linestyle="--", linewidth=2.2, label="A* 全局规划航线", zorder=4)

    # 小车实际行驶轨迹
    rx = [p[0] for p in robot_trajectory_2d]
    ry = [p[1] for p in robot_trajectory_2d]
    ax_2d.plot(rx, ry, color="#ffeb3b", linestyle="-", linewidth=3.0, label="小车实际行驶轨迹", zorder=6)

    # 标绘坡道入口与高台区域
    ax_2d.scatter([0.0], [3.0], color="#ff9100", s=180, marker="^", edgecolors="white", linewidths=1.8, label="坡道入口起点", zorder=8)
    ax_2d.scatter([start_point[0]], [start_point[1]], color="#00e676", s=160, marker="o", edgecolors="white", linewidths=1.8, label="出发起点", zorder=8)
    ax_2d.scatter([goal_point[0]], [goal_point[1]], color="#ff1744", s=200, marker="*", edgecolors="white", linewidths=1.8, label="高台终点", zorder=8)

    # 小车最终位姿与轮廓
    final_pos = sim.get_robot_position()
    final_yaw = sim.get_robot_yaw()
    ax_2d.arrow(
        final_pos[0], final_pos[1],
        0.5 * math.cos(final_yaw), 0.5 * math.sin(final_yaw),
        head_width=0.25, head_length=0.20, fc="#ffeb3b", ec="white", linewidth=1.5, zorder=9
    )
    circle = plt.Circle((final_pos[0], final_pos[1]), elevation_map.robot_radius, color="#ffeb3b", fill=False, linewidth=2.0, label="小车车体轮廓", zorder=8)
    ax_2d.add_patch(circle)

    status_str = " [成功登顶高台并驻车]" if reached_goal else " [自主巡航爬坡中]"
    ax_2d.set_title(f"2.5D 地形高程感知与自主爬坡寻路{status_str}", color="white", fontsize=13, pad=10)
    ax_2d.set_xlabel("世界坐标 X (米)", color="#b0b8c4", fontsize=10)
    ax_2d.set_ylabel("世界坐标 Y (米)", color="#b0b8c4", fontsize=10)
    ax_2d.tick_params(colors="#8892a0")
    ax_2d.set_xlim(-9.0, 9.0)
    ax_2d.set_ylim(-9.0, 9.0)
    ax_2d.grid(True, color="#1e232d", linestyle=":", alpha=0.6)
    leg = ax_2d.legend(loc="upper right", facecolor="#161b22", edgecolor="#30363d", labelcolor="white", fontsize=8.5)
    leg.get_frame().set_alpha(0.85)

    # ---------------- 子图 2 (右上): 3D 地形坡度与立体爬坡行进轨迹 ----------------
    ax_3d = fig.add_subplot(2, 2, 2, projection="3d")
    ax_3d.set_facecolor("#06080c")
    ax_3d.xaxis.set_pane_color((0.04, 0.05, 0.08, 1.0))
    ax_3d.yaxis.set_pane_color((0.04, 0.05, 0.08, 1.0))
    ax_3d.zaxis.set_pane_color((0.04, 0.05, 0.08, 1.0))

    # 降采样绘制 3D 局部地形曲面
    step_s = 4
    x_sub = np.linspace(elevation_map.origin_x, elevation_map.origin_x + elevation_map.size_x, elevation_map.nx)[::step_s]
    y_sub = np.linspace(elevation_map.origin_y, elevation_map.origin_y + elevation_map.size_y, elevation_map.ny)[::step_s]
    xx_sub, yy_sub = np.meshgrid(x_sub, y_sub)
    zz_sub = elevation_map.elevation_array[::step_s, ::step_s]
    slope_sub = elevation_map.slope_deg_array[::step_s, ::step_s]

    surf = ax_3d.plot_surface(
        xx_sub, yy_sub, zz_sub,
        facecolors=plt.cm.coolwarm(np.clip(slope_sub / 18.0, 0, 1)),
        rstride=1, cstride=1, alpha=0.65, linewidth=0.2, antialiased=True
    )

    # 绘制小车 3D 空间立体爬坡轨迹
    rx3 = [p[0] for p in robot_trajectory_3d]
    ry3 = [p[1] for p in robot_trajectory_3d]
    rz3 = [p[2] for p in robot_trajectory_3d]
    ax_3d.plot(rx3, ry3, rz3, color="#00ffff", linestyle="-", linewidth=3.5, label="小车 3D 爬坡轨迹", zorder=10)

    # 当前位姿与高台终点
    ax_3d.scatter([final_pos[0]], [final_pos[1]], [final_pos[2]], color="#ffeb3b", s=140, marker="o", edgecolors="white", label="当前 3D 位姿", zorder=11)
    ax_3d.scatter([goal_point[0]], [goal_point[1]], [0.12], color="#ff1744", s=180, marker="*", edgecolors="white", label="高台顶面终点", zorder=11)

    ax_3d.set_title("3D 立体地形高程与自主爬坡轨迹 (Z: 0.00m -> 0.12m)", color="white", fontsize=13, pad=10)
    ax_3d.set_xlabel("X (米)", color="#b0b8c4", fontsize=9)
    ax_3d.set_ylabel("Y (米)", color="#b0b8c4", fontsize=9)
    ax_3d.set_zlabel("高程 Z (米)", color="#b0b8c4", fontsize=9)
    ax_3d.tick_params(colors="#8892a0", labelsize=8)
    ax_3d.set_xlim(-9.0, 9.0)
    ax_3d.set_ylim(-9.0, 9.0)
    ax_3d.set_zlim(-0.05, 1.5)
    ax_3d.view_init(elev=32, azim=-125)
    leg3d = ax_3d.legend(loc="upper left", facecolor="#161b22", edgecolor="#30363d", labelcolor="white", fontsize=8)
    leg3d.get_frame().set_alpha(0.85)

    # ---------------- 子图 3 (左下): 车载第一人称与高台全局监控双视讯 ----------------
    ax_cam = fig.add_subplot(2, 2, 3)
    ax_cam.set_facecolor("#06080c")
    split_line = np.full((front_frame.shape[0], 6, 3), 40, dtype=np.uint8)
    combined_video = np.hstack([front_frame, split_line, overview_frame])
    ax_cam.imshow(combined_video)
    ax_cam.set_title("车载双机位协同视讯: [左] 车载前视第一人称主驱  |  [右] 高空全局监控 (小车驻车于高台)", color="white", fontsize=12, pad=10)
    ax_cam.axis("off")

    # ---------------- 子图 4 (右下): 动力学指令、车身俯仰角与距离收敛曲线 ----------------
    ax_plot = fig.add_subplot(2, 2, 4)
    ax_plot.set_facecolor("#0a0d13")
    line1 = ax_plot.plot(time_series, vx_history, color="#00ffcc", linewidth=2.0, label="线速度 v (m/s)")
    line2 = ax_plot.plot(time_series, vw_history, color="#ff9900", linewidth=1.6, linestyle="--", label="角速度 ω (rad/s)")
    line3 = ax_plot.plot(time_series, pitch_history, color="#ffea00", linewidth=2.0, linestyle="-.", label="车体爬坡俯仰角 Pitch (°)")

    ax_plot_dist = ax_plot.twinx()
    line4 = ax_plot_dist.plot(time_series, dist_history, color="#ff3366", linewidth=2.0, linestyle=":", label="距目标残差 (m)")

    lines = line1 + line2 + line3 + line4
    labels = [l.get_label() for l in lines]
    ax_plot.legend(lines, labels, loc="upper right", facecolor="#161b22", edgecolor="#30363d", labelcolor="white", fontsize=9)

    ax_plot.set_title("底盘动力学指令、车身爬坡俯仰角与距离收敛曲线", color="white", fontsize=12, pad=10)
    ax_plot.set_xlabel("仿真时间 (秒)", color="#b0b8c4", fontsize=10)
    ax_plot.set_ylabel("控制指令与姿态角", color="#b0b8c4", fontsize=10)
    ax_plot_dist.set_ylabel("目标距离 (m)", color="#ff3366", fontsize=10)
    ax_plot.tick_params(colors="#8892a0")
    ax_plot_dist.tick_params(colors="#ff3366")
    ax_plot.grid(True, color="#1e232d", linestyle=":", alpha=0.5)

    plt.subplots_adjust(left=0.04, right=0.96, top=0.94, bottom=0.06, wspace=0.16, hspace=0.22)

    os.makedirs(os.path.dirname(os.path.abspath(output_image)), exist_ok=True)
    plt.savefig(output_image, facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close()
    print(f"[OK] 2.5D 高程与立体爬坡综合态势大图已成功导出: {output_image}")

    # 若存在工件环境变量，则动态同步镜像至工件目录（保持代码无宿主绝对路径绑定）
    artifact_dir = os.environ.get("ANTIGRAVITY_ARTIFACT_DIR")
    if artifact_dir and os.path.exists(artifact_dir):
        artifact_file = os.path.join(artifact_dir, "autonomous_navigation_dashboard.png")
        shutil.copyfile(output_image, artifact_file)
        print(f"[OK] 已动态同步镜像至工件目录: {artifact_file}")


def main():
    parser = argparse.ArgumentParser(description="移动机器人 2.5D 高程感知、立体坡道爬坡与自主导航")
    parser.add_argument("--goal", nargs=2, type=float, default=[0.0, 6.0], help="导航终点物理坐标 X Y (默认北侧高台中心 0.0 6.0)")
    parser.add_argument("--headless", action="store_true", default=False, help="启用无头模式 (默认自动启动 3D 原生可视化视口)")
    parser.add_argument("--max_steps", type=int, default=900, help="最大控制步数")
    parser.add_argument("--output", type=str, default="output/autonomous_navigation_dashboard.png", help="输出图片路径")
    args = parser.parse_args()

    run_navigation(
        goal_x=args.goal[0],
        goal_y=args.goal[1],
        headless=args.headless,
        max_steps=args.max_steps,
        output_image=args.output,
    )


if __name__ == "__main__":
    main()
