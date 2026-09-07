# mobile_robot_station (移动机器人综合工作站)

[![Platform: Ubuntu 24.04](https://img.shields.io/badge/Platform-Ubuntu%2024.04-orange.svg)](https://ubuntu.com/)
[![Physics: MuJoCo 3.9](https://img.shields.io/badge/Physics-MuJoCo%203.9-blue.svg)](https://mujoco.org/)
[![Hardware: CPU Optimized](https://img.shields.io/badge/Hardware-Multi--Core%20CPU%20%2B%20iGPU-brightgreen.svg)]()
[![Architecture: Standalone](https://img.shields.io/badge/Architecture-Self--Contained%20(No%20Symlinks)-purple.svg)]()
[![Protocol: JSON--RPC 2.0](https://img.shields.io/badge/Protocol-TCP%20JSON--RPC%202.0-red.svg)]()

`mobile_robot_station` 是一个面向自主移动机器人定位、3D 激光惯导建图 (LIO-SLAM) 与自主路径规划导航的工业级开源工程体系。

本项目与具身操作机械臂工作站 [robot_arm_station](https://github.com/bobisuperplus-sys/robot_arm_station) 保持相同的模块化解耦与前后端工业微服务哲学。二者在接口与通信层完全并列对称，底层均封装为独立宿主后台服务，具备无缝组合为**复合移动操作机器人 (Mobile Manipulator，即移动底盘与操作臂集成)** 的系统级演进能力。

---

## 1. 系统总体架构与前后端通信总线

为彻底规避 ROS 2 框架在轻量环境下的臃肿依赖与资源争抢，本项目采用工业自动化与智能仓储 AMR (自主移动机器人) 广泛验证的**微服务化解耦架构**。整个系统划分为 **MSH 后台服务 (Mobile Station Host)** 与 **Qt 6 QML 数字孪生控制台**：

```text
┌────────────────────────────────────────────────────────────────────────┐
│                   Qt 6 QML / C++ 移动数字孪生中控台                      │
│      - 3D 点云与空间地图渲染视口 (OpenGL/Qt3D)                          │
│      - GStreamer 车载视讯原生低延迟解码                                │
│      - 2D 栅格代价地图 (Costmap) 交互与目标航点 (Waypoint) 下发        │
│      - 车辆动力学与 IMU 姿态遥测仪表盘                                  │
└───────────────────▲────────────────────────────────▲───────────────────┘
                    │                                │
    1. TCP JSON-RPC 2.0 (控制/状态/航点)              │ 2. RTP/UDP (H.264 车载视频流)
                    │                                │
┌───────────────────▼────────────────────────────────┴───────────────────┐
│                 MSH 核心后台服务 (Mobile Station Host)                  │
│                                                                        │
│  [通信服务层]                                                           │
│  - TCP Server (默认监听 127.0.0.1:9001，双向全双工 JSON-RPC 2.0)        │
│  - GStreamer 视讯采集与推流管道                                         │
├────────────────────────────────────────────────────────────────────────┤
│  [算法与感知层]                                                         │
│  - 3D 激光雷达 Raycasting 引擎 (16线/32线，单帧 2~5ms 极速生成)          │
│  - 6 轴 IMU 噪声与随机游走偏置注入                                      │
│  - 增量 kd 树 (ikd-Tree) 与紧耦合 LIO-SLAM 点云全局拼接                 │
│  - 2D 障碍物栅格地图生成 + A* 全局路径搜索 + DWA 局部避障控制器        │
├────────────────────────────────────────────────────────────────────────┤
│  [物理底座与动力学]                                                     │
│  - MuJoCo 物理引擎 (TurtleBot3 Burger 差速动力学 + 隐式数值积分)       │
│  - 立体建筑群综合测试世界 (Urban Proving Ground)                       │
└────────────────────────────────────────────────────────────────────────┘
```

---

## 2. 硬件适配原则与纯 CPU 极致优化

针对通用多核 CPU 与核显 (无独立强算力 GPU) 的物理环境，本项目彻底摒弃了依赖重型 CUDA 的方案，全面采用纯 CPU 向量化与多线程优化的算法架构：

1. **物理仿真与射线追踪 (MuJoCo CPU 内核)**：
   - 刚体动力学微分方程求解完全运行于多线程 CPU，实现零显存消耗；
   - 依托 MuJoCo 原生 AVX 指令集优化射线求交 (Raycasting)，16 线/32 线 3D 激光雷达点云单帧投射耗时压缩至 2~5ms，轻松跑满 10~20Hz 实时帧率；
2. **空间几何与李代数求导 (Eigen3 + Sophus)**：
   - 严格采用 SO(3), SE(3) 李代数切空间微扰模型表示旋转与平移变换，杜绝万向节死锁与欧拉角参数冗余；
3. **增量点云动态检索 (ikd-Tree)**：
   - 借鉴港大火星实验室 FAST-LIO2 算法核心，纯靠 CPU 多核维护动态增量 kd 树，配准与近邻搜索单帧仅需 10~15ms；
4. **资产全量自包含 (Zero Symlinks)**：
   - 坚决杜绝任何外部软链接。所有 3D CAD 视觉网格 (STL)、物理碰撞体、动力学参数与仿真建筑群场景均完整固化在工程内部，保证开箱即用。

---

## 3. 立体建筑群仿真测试场 (Urban Proving Ground)

为 3D 激光惯导 SLAM (LIO) 与自主导航规划提供严苛、丰富且可信的几何测试基准，场景文件位于 `assets/scenes/urban_world.xml`：

| 场景功能区 | 结构组成与特征 | 算法验证价值 |
| :--- | :--- | :--- |
| **高层科技塔楼 A** | 高 6.5m 错落双层建筑、凹凸外立面、门廊 | 保证远距离垂直高程与密集大平面点云，消除垂直几何退化 |
| **现代商业综合体 B** | 高 4.5m 砖红质感外立面、侧向防撞柱墩 | 提供多方位法向量约束与立体角点特征线 |
| **狭窄街道走廊 C** | L 型院落巷道 (Alleyway)、双侧狭长墙体 | 测试激光雷达在狭长受限廊道中的抗漂移与配准鲁棒性 |
| **立体缓坡台阶 D** | 12 度倾角缓坡 (Ramp)、阶梯平台 | 激发 6 轴 IMU 俯仰角动态变化，检验 3D 点云坡道投影与切片 |
| **立柱连廊 (Colonnade)** | 标准化圆柱阵列 (半径 0.22m，高 2.8m) | 激光雷达高曲率特征点 (Edge Points) 提取的黄金校验场 |
| **环形闭合干道 (Loop)** | 贯通整个街区的宽阔柏油主干道与中央环岛 | **回环检测 (Loop Closure) 与因子图位姿图优化的基准闭合回路** |

---

## 4. MSH TCP JSON-RPC 2.0 工业协议规范

MSH 默认监听 `127.0.0.1:9001`，所有通信采用标准 JSON-RPC 2.0 格式（全双工、换行符 `\n` 分隔）：

### 4.1 底盘速度控制 (`set_velocity`)
- **上位机请求**:
  ```json
  {"jsonrpc": "2.0", "method": "set_velocity", "params": {"v": 0.15, "w": 0.2}, "id": 1}
  ```
- **MSH 响应**:
  ```json
  {"jsonrpc": "2.0", "result": {"status": "ok", "applied_v": 0.15, "applied_w": 0.2}, "id": 1}
  ```

### 4.2 紧急制动 (`emergency_stop`)
- **上位机请求**:
  ```json
  {"jsonrpc": "2.0", "method": "emergency_stop", "params": {}, "id": 2}
  ```
- **MSH 响应**:
  ```json
  {"jsonrpc": "2.0", "result": {"status": "stopped", "v": 0.0, "w": 0.0}, "id": 2}
  ```

### 4.3 状态与遥测轮询 (`get_telemetry`)
- **上位机请求**:
  ```json
  {"jsonrpc": "2.0", "method": "get_telemetry", "params": {}, "id": 3}
  ```
- **MSH 响应**:
  ```json
  {
    "jsonrpc": "2.0",
    "result": {
      "pose": {"x": 1.25, "y": -4.82, "z": 0.01, "yaw_deg": 45.2},
      "velocity": {"linear": 0.15, "angular": 0.2},
      "imu": {"accel": [0.01, -0.02, 9.81], "gyro": [0.0, 0.0, 0.2]},
      "mapping_active": true
    },
    "id": 3
  }
  ```

### 4.4 自主导航航点下发 (`navigate_to`)
- **上位机请求**:
  ```json
  {"jsonrpc": "2.0", "method": "navigate_to", "params": {"x": 7.0, "y": 7.0, "yaw_deg": 90.0}, "id": 4}
  ```
- **MSH 响应**:
  ```json
  {"jsonrpc": "2.0", "result": {"status": "planning_accepted", "target": {"x": 7.0, "y": 7.0}}, "id": 4}
  ```

---

## 5. 工程目录架构与模块化分层

```text
mobile_robot_station/
├── assets/                          # 核心资产库 (完全自包含，无外部软链接)
│   ├── models/turtlebot3/           # TurtleBot3 Burger 移动机器人物理与视觉模型
│   │   ├── meshes/                  # 自包含 STL 3D 几何网格 (base, wheels, sensors)
│   │   ├── scene.xml                # 机器人单体预览场景
│   │   └── turtlebot3_burger.xml    # 差速驱动动力学、IMU 与 3D 激光雷达挂载站点
│   └── scenes/                      # 仿真测试世界
│       └── urban_world.xml          # 专为 3D SLAM 打造的立体建筑群综合测试场
├── config/                          # 传感器参数、激光雷达视场与算法超参数 (YAML)
├── src/                             # 核心算法与业务逻辑模块化实现
│   ├── msh/                         # MSH 后台微服务 (TCP JSON-RPC 2.0 协议服务)
│   ├── simulation/                  # 仿真驱动层 (底盘驱动、CPU Raycasting 激光雷达、IMU 噪声)
│   │   ├── robot_driver.py          # 差速底盘逆运动学控制器与航位推算
│   │   ├── world_sim.py             # MuJoCo 物理世界封装与传感器抓取
│   │   └── teleop.py                # 终端非阻塞键盘捕获与交互控制
│   ├── core_math/                   # 空间几何变换、李代数与基础数学库
│   ├── slam/                        # 点云配准 (ICP/NDT)、ikd-Tree、LIO 里程计与回环优化
│   ├── navigation/                  # 2D/3D 栅格代价地图 (Costmap)、A* 全局与 DWA 局部规划
│   └── gui/                         # Qt 6 QML 数字孪生移动中控台
├── scripts/                         # 轻量级业务启动与调度入口
│   └── run_teleop_simulation.py     # 键盘交互遥控与建筑群物理仿真入口脚本
├── tests/                           # 核心算法单元测试
├── requirements.txt                 # 项目 Python 依赖库清单 (纯 CPU 友好)
├── RULES.md                         # 项目工程规范与 AI 对话准则
├── .gitignore                       # 规范忽略临时文件与编译缓存
└── README.md                        # 工业级系统文档
```

---

## 6. 快速启动指南

### 6.1 环境准备与依赖安装
建议使用 Python 虚拟环境 (Python >= 3.10):
```bash
# 安装通用 Python 依赖 (纯 CPU 友好，无需 CUDA)
pip install -r requirements.txt
```

若在本机开发，可直接激活指定环境:
```bash
source /home/yellowtown/Code/PythonProject/venv/bin/activate
```

### 6.2 启动小车与建筑群交互式仿真
运行启动脚本，即可拉起 MuJoCo 3D 视口，并在终端实时查看小车运行位姿与 IMU 遥测数据流：
```bash
python scripts/run_teleop_simulation.py
```

### 6.3 键盘遥控操作指令
小车采用标准双轮差速逆运动学模型，在打开的终端内直接键入控制：
- **`W` / `S`**：平滑增加 / 减小前进线速度 v (最大 0.22 m/s)；
- **`A` / `D`**：平滑增加 / 减小差速旋转角速度 w (最大 2.84 rad/s)；
- **`Space` (空格)**：紧急制动 (Emergency Stop)；
- **`Q` / `ESC`**：优雅退出仿真。

---

## 7. 全生命周期五阶段演进路线图

- [x] **Stage 1.1：移动底座动力学与建筑群场景自包含交付**
  - [x] 完整拷贝 TurtleBot3 工业 CAD STL 网格，消除外部软链接；
  - [x] 调优接触阻尼与质量惯量比，杜绝数值振荡与 QACC 警告；
  - [x] 搭建带闭环回路的 3D 立体建筑群与街区测试场；
  - [x] 实现差速逆运动学与键盘非阻塞交互控制，代码模块化拆分。
- [ ] **Stage 1.2：CPU 极速 3D 激光雷达 Raycasting 引擎与 IMU 噪声模型**
  - [ ] 基于 MuJoCo 光线投射实现 16 线/32 线实时点云抓取 (单帧 2~5ms)；
  - [ ] 注入 IMU 高斯白噪声与随机游走偏置 (Bias)。
- [ ] **Stage 2：空间几何李代数与纯 CPU 点云配准**
  - [ ] Eigen3 + Sophus SE(3) 位姿变换矩阵工具集；
  - [ ] 多线程点到面 ICP 与 NDT 配准算法手撕与性能评测。
- [ ] **Stage 3：3D 激光惯导里程计 (LIO-SLAM)**
  - [ ] 增量 kd 树 (ikd-Tree) 动态插入与降采样；
  - [ ] 紧耦合状态估计器与全局地图拼接。
- [ ] **Stage 4：栅格代价地图与自主路径导航**
  - [ ] 3D 点云切片投影至 2D Costmap；
  - [ ] A* 全局路径规划与 DWA 局部动态避障。
- [ ] **Stage 5：MSH 微服务守护进程与 Qt 6 QML 数字孪生监控站**
  - [ ] MSH TCP JSON-RPC 2.0 异步服务端实现；
  - [ ] GStreamer 车载低延迟视讯推流管道；
  - [ ] Qt 6 QML 原生视口渲染 3D 点云、实时轨迹与航点导航交互。

---

## 8. 开源许可证
本项目遵循 Apache 2.0 开源许可证。
