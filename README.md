# 移动机器人自主定位建图与数字孪生工作站 (Mobile Robot Station)

本项目为一个独立的工业级移动机器人自主导航与空间感知平台，实现差速移动底盘基于 3D 激光雷达与 6 轴 IMU 惯导传感器，在复杂立体建筑群环境中完成实时光线投射 (Raycasting)、点云特征配准、增量式全局 3D 建图、2D 代价栅格切片生成与自主路径避障导航的完整控制闭环。

---

## 1. 系统架构与工程目录

本项目严格遵循工业级高内聚、低耦合的模块化分层设计，与机械臂具身操作工作站保持对称的微服务解耦架构：

```text
mobile_robot_station/
├── requirements.txt           # Python 核心依赖清单与环境配置指南
├── RULES.md                   # 项目工程规范与 AI 交互行为准则
│
├── assets/                    # 核心资产库 (完全自包含，无外部软链接)
│   ├── models/turtlebot3/     # TurtleBot3 差速小车 CAD 网格、动力学与传感器配置
│   │   ├── meshes/            # 自包含 STL 3D 几何网格 (底盘、车轮、传感器)
│   │   ├── scene.xml          # 机器人单体独立预览场景
│   │   └── turtlebot3_burger.xml # 差速驱动、IMU与激光雷达挂载站点
│   └── scenes/                # 仿真测试世界资产
│       └── urban_world.xml    # 专为 3D SLAM 打造的立体建筑群综合测试场
│
├── src/                       # 核心算法与业务逻辑模块
│   ├── simulation/            # 物理动力学底座 (MuJoCo 引擎、CPU 极速 Raycasting、IMU 噪声)
│   │   ├── robot_driver.py    # 差速底盘逆运动学控制器与轮速映射
│   │   ├── world_sim.py       # MuJoCo 物理环境封装与传感器遥测提取
│   │   └── teleop.py          # 终端非阻塞键盘事件交互监听器
│   │
│   ├── core_math/             # 空间几何代数与李代数核心库 (SE3/SO3 切空间微扰求导)
│   ├── slam/                  # 激光惯导里程计内核 (点云配准、增量式 ikd-Tree、全局建图)
│   ├── navigation/            # 2D 代价栅格切片生成、A* 全局路径规划与 DWA 局部避障
│   └── msh/                   # MSH (Mobile Station Host) 后台主控微服务与通信总线
│
├── scripts/                   # 业务启动与功能验证入口脚本
│   └── run_teleop_simulation.py # 键盘交互遥控与建筑群物理仿真入口脚本
│
├── tests/                     # 核心算法单元测试用例
└── qt_client/                 # 工业数字孪生上位机客户端工程 (C++ / Qt 6 QML)
    └── mobile_console/        # Cyber 赛博工业风格控制台 (3D点云视口、地图导航、遥测仪表盘)
```

---

## 2. 快速开始与环境安装 (一键配置)

### 步骤一：安装 Python 依赖
推荐在独立的虚拟环境 (Virtualenv / Conda) 中运行：

```bash
# 激活 Python 虚拟环境后执行 (纯 CPU 友好，无需 CUDA)
pip install -r requirements.txt
```

若在本机开发，可直接激活指定专属环境：
```bash
source /home/yellowtown/Code/PythonProject/venv/bin/activate
```

### 步骤二：安装系统级 GStreamer 视讯多媒体库 (Linux 环境)
工作站采用 GStreamer RTP H.264 UDP 硬件/软件编码推流车载视讯画面，请确保宿主机已安装核心插件：

```bash
sudo apt update && sudo apt install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav
```

---

## 3. 工作站运行指南

### 1. 启动核心仿真与遥测演示 (Simulation & Teleop)
拉起 MuJoCo 3D 物理引擎，加载立体建筑群测试世界，并通过终端键盘进行实时差速遥控：

```bash
python scripts/run_teleop_simulation.py
```
- **控制按键**：`W/S` 加减速前进后退、`A/D` 差速原地转向、`Space` 紧急制动刹车、`Q/ESC` 安全退出。

### 2. 启动 GStreamer 视讯推流与移动仿真 (Streaming Simulation)
拉起双机位实时视讯推流，将车载感知相机 (UDP: 5002) 与全局监控相机 (UDP: 5004) 通过 H.264 RTP 广播至上位机：

```bash
python scripts/run_streaming_simulation.py
```

### 3. 启动统一核心主控服务 (MSH Host Server)
拉起后台 MSH 守护微服务，负责物理仿真步进、3D 激光点云生成、车载相机推流并开放 TCP:9001 JSON-RPC 控制总线：

```bash
python src/msh/msh_server.py
```

### 4. 启动数字孪生上位机控制台 (Qt 6 Client)
在另一个终端中启动已编译就绪的上位机界面：

```bash
./qt_client/mobile_console/build/appmobile_console
```
上位机启动后将自动建立双向遥测信道，挂载低延迟车载视频画面，并实时以 3D 方式渲染点云与历史运动轨迹。

---

## 4. 核心工作流与技术实现

移动机器人的自主建图与导航全任务由高可靠性有限状态机 (FSM) 驱动，包含以下 5 个关键阶段：

1. **激光感知与光线求交 (Raycast Acquisition)**:
   - 车载 3D 激光雷达发射中心 (`lidar_site`) 基于 MuJoCo AVX 向量化光线投射，以 2 - 5ms 的极速完成 16 线 360 度全向扫描，实时输出密集的建筑物与道路空间交点；
2. **点云配准与增量空间建图 (LIO Mapping)**:
   - 融合 6 轴 IMU 预测位姿，采用多线程点到面 ICP 与增量式 kd 树 (ikd-Tree) 动态数据结构，在纯 CPU 算力下以 10 - 15ms 快速拼接全局 3D 点云地图；
3. **2D 代价栅格切片生成 (Costmap Projection)**:
   - 提取 3D 点云在底盘离地净空高度范围内的障碍点，二维正交投影并膨胀生成包含不可通行区与安全缓冲区的 2D 占用栅格代价地图；
4. **全局与局部自主避障规划 (Plan & Avoid)**:
   - 响应导航指令，基于 A* 算法搜索全局无碰撞拓扑路径；局部采用动态窗口法 (DWA) 结合当前车速与周围动态障碍物实时解算最优安全线速度与角速度 $(v, \omega)$；
5. **目标航点精准停靠与状态复位 (Arrive & Home)**:
   - 差速底盘沿平滑轨迹自主驶向目标位姿 $(X, Y, \text{Yaw})$，停靠精度控制在厘米级；到位后自动切入保持待机状态，向上位机回传执行报告与位姿闭环结果。

---

## 5. MSH 工业协议与双通道解耦总线 (Communication Bus)

系统采用通信通道物理分离设计，兼顾高吞吐多媒体与轻量高可靠控制：

1. **控制与遥测总线 (TCP JSON-RPC 2.0，默认端口: 9001)**:
   - 采用标准 JSON-RPC 2.0 报文协议，支持底盘速度控制 (`set_velocity`)、紧急刹车 (`emergency_stop`)、高频空间遥测轮询 (`get_telemetry`) 以及目标航点导航下发 (`navigate_to`)；
2. **车载低延迟视讯管道 (GStreamer RTP/UDP，默认端口: 5002)**:
   - 底盘前向感知相机捕获的图像帧直接压入 GStreamer 管道，经由 H.264 硬件/软件编码封包为 RTP 数据报广播，实现上位机视口秒级挂载与超低延迟呈现。

---

## 6. 开源许可证
本项目遵循 Apache 2.0 开源许可证。
