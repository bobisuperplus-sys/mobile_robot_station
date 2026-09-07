# 移动机器人自主定位建图与数字孪生工作站 (Mobile Robot Station)

本项目为一个独立的工业级移动机器人自主定位建图与空间导航平台，实现差速移动底盘基于 3D 激光雷达与 6 轴 IMU 惯导传感器，在复杂立体建筑群仿真环境中完成实时光线投射 (Raycasting)、惯性预测、点云特征配准、增量式全局 3D 建图 (LIO-SLAM)、2D 障碍物代价栅格投影、自主路径避障导航以及双机位低延迟 H.264 视讯推流的完整控制闭环。

---

## 1. 系统架构与工程目录

本项目严格遵循工业级高内聚、低耦合的模块化分层设计，全工程物理目录与文档清单保持严格同步：

```text
mobile_robot_station/
├── requirements.txt                   # Python 核心依赖清单与环境配置指南
├── RULES.md                           # 项目工程规范与 AI 交互行为准则
├── README.md                          # 项目工程介绍、架构目录与运行指南
├── .gitignore                         # Git 版本控制忽略规则配置
│
├── assets/                            # 核心资产库 (完全自包含，无外部软链接)
│   ├── models/turtlebot3/             # TurtleBot3 差速小车 CAD 网格、动力学与传感器配置
│   │   ├── meshes/                    # 自包含 STL/DAE/JPG 几何网格与贴图 (底盘、车轮、雷达)
│   │   │   ├── bases/                 # 车体底盘与底板 STL 模型 (burger_base 等)
│   │   │   ├── sensors/               # 传感器几何外壳模型 (lds.stl, astra, r200)
│   │   │   └── wheels/                # 驱动轮 STL 模型 (left_tire, right_tire)
│   │   ├── scene.xml                  # 机器人单体独立预览场景 (MuJoCo MJCF)
│   │   └── turtlebot3_burger.xml      # 差速驱动、6轴 IMU 与 16线激光雷达核心动力学模型
│   └── scenes/                        # 仿真测试世界资产
│       └── urban_world.xml            # 专为 3D SLAM 打造的立体建筑群综合测试场 (坡道、连廊、立体楼宇)
│
├── config/                            # 系统与传感器参数配置目录
│   └── sensors.yaml                   # 激光雷达扫描线束/视场角及 IMU 高斯白噪声与漂移物理参数表
│
├── src/                               # 核心算法与业务逻辑源码
│   ├── __init__.py                    # 顶层包标识
│   ├── simulation/                    # 物理动力学与传感器仿真底座 (MuJoCo、CPU Raycasting、视讯推流)
│   │   ├── __init__.py                # 模块导出定义
│   │   ├── platform_compat.py         # Linux 桌面 (X11/Wayland) 图形环境兼容与警告拦截器
│   │   ├── robot_driver.py            # 差速底盘逆运动学控制器与轮速映射解算器
│   │   ├── imu_sim.py                 # 6 轴 MEMS 惯性测量单元仿真器 (角速度/线加速度噪声与偏置游走)
│   │   ├── lidar_sim.py               # 16 线 360° 极速 CPU Raycasting 光线求交与 3D 点云发生器
│   │   ├── streamer.py                # 基于 GStreamer 的 H.264 RTP/UDP 双机位低延迟视讯推流管道
│   │   ├── teleop.py                  # 终端非阻塞键盘事件监听与速度增量遥控器
│   │   └── world_sim.py               # MuJoCo 物理环境生命周期管理与多机位离屏渲染集成器
│   │
│   ├── core_math/                     # 空间几何代数与李代数核心库 (SE3/SO3 切空间微扰与李括号)
│   │   └── __init__.py                # 模块导出定义
│   ├── slam/                          # 3D 激光惯导紧耦合里程计 (点云配准、增量式 ikd-Tree、全局建图)
│   │   └── __init__.py                # 模块导出定义
│   ├── navigation/                    # 自主路径规划与局部避障 (2D Costmap 投影、A* 规划、DWA 避障)
│   │   └── __init__.py                # 模块导出定义
│   └── msh/                           # MSH (Mobile Station Host) 后台主控微服务与通信总线
│       └── __init__.py                # 模块导出定义
│
├── scripts/                           # 业务启动与功能验证入口脚本
│   ├── run_teleop_simulation.py       # 键盘交互式差速小车遥控与城市建筑群物理仿真主入口
│   ├── run_streaming_simulation.py    # GStreamer 双机位 (车载前向 + 全局监控) H.264 视讯推流仿真入口
│   └── view_lidar_scan.py             # 独立激光雷达点云捕获与测距单帧交互可视化验证工具
│
├── tests/                             # 核心算法与仿真模块单元测试套件
│   ├── __init__.py                    # 测试包标识
│   └── test_sensors.py                # 激光雷达光线求交、IMU 高斯噪声与平台兼容性自动化测试集
│
└── qt_client/                         # 工业数字孪生上位机客户端工程 (C++ / Qt 6 QML)
    └── mobile_console/                # 基于 Qt 6 构建的 Cyber 赛博工业风格移动机器人控制台
        ├── CMakeLists.txt             # Qt 6 C++ / QML 项目 CMake 构建配置文件
        ├── main.cpp                   # 上位机客户端主入口
        ├── Main.qml                   # 工业控制台 QML 界面主视图
        ├── importedcontent/           # Figma to Qt 模块集成扩展目录
        └── .gitignore                 # Qt Creator 专用本地构建忽略规则
```

---

## 2. 快速开始与环境安装 (一键配置)

### 步骤一：安装 Python 依赖
推荐在独立的虚拟环境 (Virtualenv / Conda) 中运行：

```bash
# 激活 Python 虚拟环境后执行 (纯 CPU 架构友好，无需 CUDA)
pip install -r requirements.txt
```

若在本机开发，可直接激活指定专属虚拟环境：
```bash
source /home/yellowtown/Code/PythonProject/venv/bin/activate
```

### 步骤二：安装系统级 GStreamer 视讯多媒体库 (Linux 环境)
工作站采用 GStreamer RTP H.264 UDP 编码推流车载与全局视讯画面，请确保宿主机已安装核心插件：

```bash
sudo apt update && sudo apt install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav
```

### 步骤三：运行核心传感器自动化测试
验证激光雷达 Raycasting、IMU 高斯噪声发生器与平台兼容性层是否正常工作：

```bash
python -m unittest tests/test_sensors.py
```

---

## 3. 工作站运行指南

### 1. 启动核心物理仿真与键盘遥控 (Simulation & Teleop)
拉起 MuJoCo 3D 物理引擎，加载立体建筑群测试世界，并通过终端键盘进行实时差速遥控：

```bash
python scripts/run_teleop_simulation.py
```
- **控制按键**：`W/S` 加减速前进后退、`A/D` 差速原地转向、`Space` 紧急制动刹车、`Q/ESC` 安全退出。

### 2. 启动 GStreamer 双机位视讯推流仿真 (Streaming Simulation)
拉起车载感知相机 (UDP: 5002) 与全局监控相机 (UDP: 5004) 双通道 H.264 RTP 实时广播，并在后台步进物理与传感器仿真：

```bash
python scripts/run_streaming_simulation.py
```

可在另一个终端中使用 GStreamer 原生管道实时预览拉流画面：
```bash
# 预览车载前向感知画面 (5002 端口)
gst-launch-1.0 -v udpsrc port=5002 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

### 3. 运行激光雷达扫描与点云验证工具 (LiDAR Scan Inspector)
在终端独立执行激光雷达单帧光线求交测试，输出 16 线点云统计信息与终端 ASCII 极坐标雷达切片：

```bash
python scripts/view_lidar_scan.py
```

### 4. 启动数字孪生上位机控制台 (Qt 6 Client)
上位机工程基于 Qt 6 (C++ / QML) 开发，可在 Qt Creator 中打开 `qt_client/mobile_console/CMakeLists.txt` 构建运行，或在终端编译后执行：

```bash
# 启动构建完成的上位机控制台
./qt_client/mobile_console/build/appmobile_console
```
上位机启动后将自动建立双向遥测信道，秒级挂载低延迟车载推流画面，并以 3D 方式渲染点云与历史运动轨迹。

---

## 4. 核心工作流与技术实现

移动机器人的自主建图与导航全任务由高可靠性有限状态机 (FSM) 驱动，包含以下 5 个关键阶段：

1. **激光感知与光线求交 (Raycast Acquisition)**:
   - 车载 3D 激光雷达 (`lidar_site`) 基于 MuJoCo 底层极速光线投射，采用单线程密集内存求交结合局部射线查找表 (LUT) 优化，单帧 (16线 × 120方位角，共 1920 束光线) 仅需约 30ms，稳定输出建筑物、地面与障碍物的密集交点云；
2. **惯导测量与运动先验 (IMU Mechanization & Pre-integration)**:
   - 6 轴高频 IMU 模拟真实世界传感器特性，注入高斯白噪声与离散随机游走零偏，在激光雷达两帧扫描间隙提供高精度的角速度与加速度积分先验位姿，消除动态运动畸变；
3. **点云配准与增量空间建图 (LIO Mapping & ikd-Tree)**:
   - 采用多线程点到面 ICP 与增量式 kd 树 (ikd-Tree) 动态数据结构，在纯 CPU 算力下以 10 - 15ms 快速更新局部地图并拼接全局 3D 稠密点云；
4. **2D 代价栅格切片生成 (Costmap Projection)**:
   - 提取 3D 点云在底盘离地净空高度范围内的障碍点，二维正交投影并膨胀生成包含不可通行区与安全缓冲区的 2D 占用栅格代价地图；
5. **全局规划与局部自主动态避障 (Plan & Avoid)**:
   - 响应导航指令，基于 A* 算法在代价地图上搜索全局最短拓扑路径；局部采用动态窗口法 (DWA) 结合当前车速与周围动态障碍物实时解算最优安全线速度与角速度 $(v, \omega)$，实现厘米级精准停靠。

---

## 5. 通信总线与视讯流规范

系统采用数据流与控制流物理隔离的解耦总线设计，兼顾高吞吐多媒体与高实时控制：

1. **控制与遥测总线 (TCP JSON-RPC 2.0，默认端口: 9001)**:
   - 采用标准 JSON-RPC 2.0 报文协议，支持底盘速度控制 (`set_velocity`)、紧急刹车 (`emergency_stop`)、高频空间遥测轮询 (`get_telemetry`) 以及目标航点导航下发 (`navigate_to`)；
2. **车载前向低延迟视讯流 (GStreamer RTP/UDP，默认端口: 5002)**:
   - 车载前向相机捕获的图像帧直接压入 GStreamer 管道，经由 H.264 软件/硬件编码封包为 RTP 数据报广播，提供极低延迟的车载第一视角；
3. **全局态势监控视讯流 (GStreamer RTP/UDP，默认端口: 5004)**:
   - 第三人称跟随/鸟瞰监控相机推流，便于上位机操控者宏观掌握机器人在立体街区中的全景运行态势。

---

## 6. 开源许可证

本项目遵循 Apache 2.0 开源许可证。
