# -*- coding: utf-8 -*-
"""
移动机器人全车功能能力清单与状态定义中心 (Robot Capabilities Manifest)

核心定位：
面向上位机操作员 (Qt 控制台)、Web 调度系统与终端开发者，统一定义、动态自省并发布移动机器人
所具备的完整硬件配备、核心算法能力、受支持的运行模式与标准 RPC 服务接口。
"""

from typing import Any, Dict, List


def get_robot_capabilities() -> Dict[str, Any]:
    """
    获取移动机器人标准化全车功能能力清单 (Capabilities Manifest)
    以结构化字典呈现，完全兼容 JSON 序列化
    """
    return {
        "system": {
            "robot_name": "TurtleBot3-Burger-Station",
            "model": "turtlebot3_burger",
            "version": "2.0.0",
            "architecture": "Differential Drive + 3D LiDAR + 6-DOF IMU",
            "communication_protocol": "TCP JSON-RPC 2.0",
            "default_rpc_port": 9001,
            "supported_modes": [
                "MANUAL",       # 人工遥控模式 (Teleoperation)
                "MAPPING",      # 3D SLAM 建图模式 (LIO-SLAM Mapping)
                "NAVIGATION",   # 2.5D 高程感知自主导航模式 (Autonomous Navigation)
                "ESTOP",        # 紧急制动停机模式
            ],
            "description": "工业级全栈移动机器人仿真与数字孪生自主作业工作站",
        },
        "capabilities": {
            "teleop": {
                "name": "底盘动力学与遥控驱动",
                "enabled": True,
                "description": "基于差速逆运动学解算轮速映射，支持平滑线速度/角速度调控与急停保护",
                "methods": [
                    {
                        "method": "teleop.drive",
                        "params": {"linear_x": "float (m/s, max 0.60)", "angular_z": "float (rad/s, max 3.5)"},
                        "description": "下发底盘目标运动线速度与角速度指令",
                    },
                    {
                        "method": "teleop.emergency_stop",
                        "params": {},
                        "description": "紧急抱闸制动，清除当前所有运动指令",
                    },
                ],
                "kinematics": {
                    "wheel_radius_m": 0.033,
                    "wheel_base_m": 0.160,
                    "max_linear_vel_mps": 0.60,
                    "max_angular_vel_radps": 3.50,
                    "max_motor_torque_nm": 2.50,
                },
            },
            "sensors": {
                "name": "多源物理传感器配置",
                "enabled": True,
                "description": "车载与环境多模态高频物理传感器组",
                "devices": [
                    {
                        "id": "lidar_3d",
                        "name": "16 线全向 3D 激光雷达",
                        "type": "Raycasting LiDAR",
                        "vertical_beams": 16,
                        "horizontal_fov_deg": 360.0,
                        "range_m": [0.20, 25.0],
                        "freq_hz": 10,
                    },
                    {
                        "id": "imu_6axis",
                        "name": "6 轴高频 MEMS 惯性测量单元",
                        "type": "Gyroscope + Accelerometer",
                        "noise_model": "Gaussian White Noise + Random Walk Bias",
                        "freq_hz": 200,
                    },
                    {
                        "id": "front_cam",
                        "name": "车载前视第一人称感知相机",
                        "type": "RGB Driver FPV",
                        "resolution": [640, 480],
                        "stream_url": "udp://127.0.0.1:5002",
                        "codec": "H.264 RTP",
                    },
                    {
                        "id": "overview_cam",
                        "name": "高空全局第三人称监控相机",
                        "type": "RGB Digital Twin Overview",
                        "resolution": [640, 480],
                        "stream_url": "udp://127.0.0.1:5004",
                        "codec": "H.264 RTP",
                    },
                ],
            },
            "slam": {
                "name": "3D 激光惯导紧耦合建图 (LIO-SLAM)",
                "enabled": True,
                "description": "基于点到面高斯-牛顿 ICP 与 6 轴 IMU 运动学中点积分，实现实时位姿估计与稠密空间点云构建",
                "methods": [
                    {
                        "method": "slam.start_mapping",
                        "params": {},
                        "description": "启动后台 3D LIO-SLAM 实时里程计与点云拼接建图",
                    },
                    {
                        "method": "slam.stop_mapping",
                        "params": {},
                        "description": "停止当前建图并冻结全局点云地图",
                    },
                    {
                        "method": "slam.save_map",
                        "params": {
                            "map_name": "string",
                            "description": "string (可选)",
                        },
                        "description": "将建图产物一键存盘为持久化地图包 (包含 .ply 点云、.npy 高程矩阵与 .yaml 元数据)",
                    },
                    {
                        "method": "slam.get_map_stats",
                        "params": {},
                        "description": "获取当前 SLAM 点云数量、配准残差与位姿估计",
                    },
                ],
            },
            "map_manager": {
                "name": "工业级地图存储与管理",
                "enabled": True,
                "description": "标准地图存储目录检索、点云读取、2.5D 高程矩阵反序列化与地图删除",
                "methods": [
                    {
                        "method": "map.list_maps",
                        "params": {},
                        "description": "列举存储库中所有可用的历史地图清单",
                    },
                    {
                        "method": "map.load_map",
                        "params": {"map_name": "string"},
                        "description": "载入指定地图包并装配至导航系统",
                    },
                    {
                        "method": "map.delete_map",
                        "params": {"map_name": "string"},
                        "description": "从存储库中安全删除指定地图包",
                    },
                ],
            },
            "navigation": {
                "name": "2.5D 高程立体感知与自主寻路避障",
                "enabled": True,
                "description": "基于二维空间差分坡度估计、局部台阶高度阻断、A* 全局立体寻路与 DWA 动态窗口局部避障",
                "methods": [
                    {
                        "method": "nav.navigate_to",
                        "params": {
                            "goal_x": "float",
                            "goal_y": "float",
                            "goal_yaw": "float (可选)",
                            "via_points": "list of [x, y] (可选安全引导航点)",
                        },
                        "description": "下发全局导航目标点并启动自主规划与越障爬坡",
                    },
                    {
                        "method": "nav.cancel_goal",
                        "params": {},
                        "description": "取消当前正在执行的导航任务并平稳刹车",
                    },
                    {
                        "method": "nav.get_status",
                        "params": {},
                        "description": "获取当前导航状态机枚举 (IDLE, PLANNING, NAVIGATING, ARRIVED, BLOCKED, CANCELLED)",
                    },
                    {
                        "method": "nav.get_trajectory",
                        "params": {},
                        "description": "获取当前 A* 全局规划航线与小车已行驶的 2D/3D 轨迹坐标",
                    },
                ],
                "elevation_features": {
                    "slope_analysis": "2D Spatial Gradient arctan|grad z|",
                    "step_detection": "3x3 Neighborhood Height Disparity",
                    "smooth_ramp_clearance": "Continuous Traversability Cost (1~120)",
                    "lethal_obstacle_blocking": "Lethal Barrier (254) with Euclidean Safety Inflation",
                },
            },
            "telemetry": {
                "name": "高频状态遥测与姿态监控",
                "enabled": True,
                "description": "小车 6-DOF 空间位姿、线速度/角速度、爬坡俯仰角 Pitch 与距离收敛遥测",
                "methods": [
                    {
                        "method": "telemetry.get_pose",
                        "params": {},
                        "description": "获取小车瞬时位置 (x, y, z) 与偏航角 yaw",
                    },
                    {
                        "method": "telemetry.get_full_status",
                        "params": {},
                        "description": "获取集成位姿、线速度、角速度、爬坡俯仰角、传感器状态与目标残差的完整综合数据",
                    },
                ],
            },
            "streaming": {
                "name": "双机位视讯广播推流",
                "enabled": True,
                "description": "基于 GStreamer RTP/UDP 的 H.264 双通道实时低延迟视讯推流广播",
                "channels": [
                    {"channel": "front_cam", "port": 5002, "protocol": "rtp/h264"},
                    {"channel": "overview_cam", "port": 5004, "protocol": "rtp/h264"},
                ],
            },
        },
    }
