# -*- coding: utf-8 -*-
"""
3D LIO-SLAM 激光惯导实时建图与轨迹估计交互式验证程序
结合 MuJoCo 真实物理世界，通过差速遥控小车漫游建筑群，在线解算 6-DOF 位姿并增量构建全局 3D 点云地图
"""

import os
import sys
import time
import argparse
import numpy as np

# 导入工程根路径
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(CURRENT_DIR)
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

# 优先导入平台兼容层
import simulation.platform_compat

try:
    import mujoco.viewer
    from simulation import (
        TurtleBot3BurgerDriver,
        UrbanWorldSimulation,
        TeleopKeyboard,
    )
    from core_math.se3 import SE3
    from core_math.so3 import SO3
    from slam.lio_odometry import LIOOdometry
except ImportError as err:
    print(f"[Error] 模块加载失败: {err}")
    print("[Tip] 请激活虚拟环境: source venv/bin/activate")
    sys.exit(1)


def save_pointcloud_ply(points: np.ndarray, file_path: str):
    """将 (N, 3) 3D 点云保存为标准 ASCII PLY 格式"""
    os.makedirs(os.path.dirname(os.path.abspath(file_path)), exist_ok=True)
    header = [
        "ply",
        "format ascii 1.0",
        f"element vertex {len(points)}",
        "property float x",
        "property float y",
        "property float z",
        "end_header",
    ]
    with open(file_path, "w", encoding="utf-8") as f:
        f.write("\n".join(header) + "\n")
        for pt in points:
            f.write(f"{pt[0]:.4f} {pt[1]:.4f} {pt[2]:.4f}\n")


def print_banner():
    banner = """
======================================================================
  Mobile Robot Station - 3D LIO-SLAM 实时建图与位姿解算控制台
======================================================================
  [控制按键说明]:
    W / S : 前进加速 / 后退减速
    A / D : 差速左转 / 差速右转
    Space : 紧急制动
    P     : 手动触发保存当前全局 3D 点云地图 (PLY 格式)
    Q/ESC : 安全退出并自动导出建图结果

  [SLAM 核心管线]:
    16 线 Raycasting -> NumPy 体素滤波 -> 点到面高斯-牛顿 ICP -> cKDTree 动态地图
======================================================================
"""
    print(banner)


def main():
    parser = argparse.ArgumentParser(description="3D LIO-SLAM 在线建图与位姿估计演示")
    parser.add_argument("--auto", action="store_true", help="启用自动循迹慢速漫游测试模式")
    parser.add_argument("--steps", type=int, default=300, help="自动模式下的仿真总步数")
    parser.add_argument("--headless", action="store_true", help="无头模式运行 (不打开 3D GUI 视口，适合自动化测试)")
    args = parser.parse_args()

    scene_path = os.path.join(PROJECT_ROOT, "assets", "scenes", "urban_world.xml")
    if not os.path.exists(scene_path):
        print(f"[Error] 未找到场景文件: {scene_path}")
        sys.exit(1)

    print_banner()

    # 1. 实例化物理仿真器与底盘驱动
    sim = UrbanWorldSimulation(scene_path)
    driver = TurtleBot3BurgerDriver()

    # 获取底盘初始真实位姿对齐 SLAM 初始化
    init_pos = sim.get_robot_position()
    init_quat = sim.get_robot_orientation()  # [w, x, y, z]
    init_so3 = SO3.from_quaternion(np.array([init_quat[1], init_quat[2], init_quat[3], init_quat[0]]))
    init_se3 = SE3(init_so3, init_pos)

    # 2. 实例化 3D LIO-SLAM 紧耦合里程计
    slam_odom = LIOOdometry(
        voxel_size=0.08,
        map_voxel_size=0.10,
        max_correspondence_dist=0.60,
        max_iterations=6,
    )
    slam_odom.reset(initial_pose=init_se3)

    last_lidar_time = 0.0
    last_hud_time = 0.0
    lidar_period = 0.05  # 激光雷达以 20 Hz 频率采集配准
    step_count = 0

    print("[Info] 正在启动 3D LIO-SLAM 引擎...")

    def run_step(viewer=None):
        nonlocal last_lidar_time, last_hud_time, step_count
        loop_start = time.time()
        sim_time = sim.data.time

        # 捕获键盘事件 (仅在有交互输入时生效)
        key = keyboard.poll_key() if not args.headless else None
        if key in ["q", "Q", "\x1b"]:
            print("\n[Info] 收到退出指令，正在终止 SLAM 并导出地图...")
            return False
        elif key in ["w", "W"]:
            driver.step_forward()
        elif key in ["s", "S"]:
            driver.step_backward()
        elif key in ["a", "A"]:
            driver.step_turn_left()
        elif key in ["d", "D"]:
            driver.step_turn_right()
        elif key in [" ", "x", "X"]:
            driver.emergency_stop()
        elif key in ["p", "P"]:
            map_pts = slam_odom.get_map_points()
            out_file = os.path.join(PROJECT_ROOT, "tests", "data", f"map_snapshot_{int(time.time())}.ply")
            save_pointcloud_ply(map_pts, out_file)
            print(f"\n[Save] 已保存当前点云地图快照 ({len(map_pts)} 点) 至: {out_file}")

        # 自动巡航测试模式支持
        if args.auto:
            step_count += 1
            if step_count < 100:
                driver.set_velocity(0.2, 0.0)
            elif step_count < 200:
                driver.set_velocity(0.15, 0.3)
            else:
                driver.set_velocity(0.2, -0.2)

            if step_count >= args.steps:
                print("\n[Info] 自动测试模式已完成指定步数。")
                return False

        # 差速驱动更新
        w_left, w_right = driver.get_wheel_angular_velocities()
        sim.apply_wheel_controls(w_left, w_right)

        # 物理引擎步进 (dt=0.002s)
        sim.step()
        dt = sim.model.opt.timestep

        # IMU 高频运动学积分
        imu_telemetry = sim.get_realistic_imu_telemetry()
        gyro = imu_telemetry["gyro_meas"]
        accel = imu_telemetry["accel_meas"]
        slam_odom.predict_with_imu(gyro, accel, dt)

        # 激光雷达低频配准 (20Hz)
        reg_ms = 0.0
        if sim_time - last_lidar_time >= lidar_period:
            lidar_t0 = time.time()
            lidar_scan = sim.get_lidar_pointcloud(return_world_frame=False)
            lidar_pts = lidar_scan["points"]
            curr_pose, reg_res = slam_odom.register_frame(lidar_pts)
            reg_ms = (time.time() - lidar_t0) * 1000.0
            last_lidar_time = sim_time

        # 视角平滑跟随小车
        if viewer is not None:
            car_pos = sim.get_robot_position()
            viewer.cam.lookat = car_pos

        # 终端 HUD 仪表盘 (以 5 Hz 刷新)
        if sim_time - last_hud_time >= 0.2:
            est_x, est_y, est_z, _, _, est_yaw = slam_odom.current_pose.to_xyz_rpy()
            true_pos = sim.get_robot_position()
            pos_error = np.linalg.norm(np.array([est_x, est_y, est_z]) - true_pos)
            map_size = len(slam_odom.global_map)

            hud = (
                f"\r[SLAM HUD] 估计位姿: X={est_x:6.2f} Y={est_y:6.2f} Z={est_z:5.2f} Yaw={np.degrees(est_yaw):6.1f}° | "
                f"真实误差: {pos_error*100:4.1f}cm | 地图点数: {map_size:5d} | 配准耗时: {reg_ms:4.1f}ms"
            )
            sys.stdout.write(hud)
            sys.stdout.flush()
            last_hud_time = sim_time

        if viewer is not None:
            viewer.sync()
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

        return True

    with TeleopKeyboard() as keyboard:
        try:
            if args.headless:
                print("[Info] 以无头模式 (Headless) 运行 SLAM 仿真...")
                while run_step(viewer=None):
                    pass
            else:
                with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
                    viewer.cam.distance = 5.0
                    viewer.cam.elevation = -25
                    viewer.cam.azimuth = -135
                    viewer.cam.lookat = init_pos.copy()
                    while viewer.is_running():
                        if not run_step(viewer=viewer):
                            break
        except Exception as e:
            print(f"\n[Error] 仿真运行异常: {e}")

    # 导出最终建图结果
    final_map = slam_odom.get_map_points()
    export_path = os.path.join(PROJECT_ROOT, "tests", "data", "urban_world_slam_map.ply")
    save_pointcloud_ply(final_map, export_path)
    print(f"\n[Success] 全局 3D 点云地图已成功保存至: {export_path}")
    print(f"[Summary] 累计注册帧数: {slam_odom.frame_count} 帧, 空间地图体素点总数: {len(final_map)} 点。")


if __name__ == "__main__":
    main()
