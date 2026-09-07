# mobile_robot_station (移动机器人综合工作站)

[![Platform: Ubuntu 24.04](https://img.shields.io/badge/Platform-Ubuntu%2024.04-orange.svg)](https://ubuntu.com/)
[![Physics: MuJoCo 3.9](https://img.shields.io/badge/Physics-MuJoCo%203.9-blue.svg)](https://mujoco.org/)
[![Hardware: CPU Optimized](https://img.shields.io/badge/Hardware-Multi--Core%20CPU%20%2B%20iGPU-brightgreen.svg)]()
[![Architecture: Standalone](https://img.shields.io/badge/Architecture-Self--Contained%20(No%20Symlinks)-purple.svg)]()

`mobile_robot_station` 是一个面向自主移动机器人定位、3D 激光惯导建图 (LIO-SLAM) 与自主路径规划导航的工业级开源工程体系。

本项目与上一阶段的机械臂具身操作工作站 [robot_arm_station](https://github.com/bobisuperplus-sys/robot_arm_station) 保持相同的模块化解耦与工业工程设计哲学。二者在接口与通信层完全并列对称，具备直接无缝组合为复合移动操作机器人 (Mobile Manipulator，即移动底盘与操作臂集成) 的系统级演进能力。

---

## 1. 架构设计哲学与强 CPU 硬件适配原则

针对多核 CPU 与核显 (无独立强算力 GPU) 的开发与部署环境，本项目彻底规避了依赖重型 CUDA 的框架，全面采用计算密度高、内存紧凑、纯 CPU 并行加速友好的高性能算法路线：

1. **物理与传感器仿真底座 (MuJoCo)**：
   - 核心物理动力学解算完全基于多线程 CPU，极度轻量且无需独显；
   - 结合 MuJoCo 原生 AVX 向量化光线投射 (Raycasting)，在纯 CPU 上以 2~5ms/帧的超低开销实时生成 16 线/32 线 3D 激光雷达点云，稳定运行于 10~20Hz 实时帧率；
2. **空间几何与李代数 (Eigen3 + Sophus)**：
   - 全面采用 SO(3), SE(3) 李群李代数切空间微扰模型表示旋转与平移，消除万向节死锁与参数冗余；
3. **增量式点云数据结构 (ikd-Tree)**：
   - 吸收 FAST-LIO2 的核心工程实践，使用纯 CPU 多核维护动态增量 kd 树 (ikd-Tree)，点云配准与近邻检索单帧控制在 10~15ms 以内；
4. **工程自包含 (Self-Contained)**：
   - 完全杜绝外部软链接 (Symlinks)。所有 3D CAD 视觉网格 (STL)、物理碰撞体、动力学参数与仿真建筑群场景均完整固化在工程内部，开箱即用，代码与资产完全合规且易于分发。

---

## 2. 3D 建筑群与场景设计 (Urban Proving Ground)

为了给 3D 激光惯导 SLAM (LIO) 与路径规划提供严苛、丰富且可信的几何测试基准，我们在 `assets/scenes/urban_world.xml` 中构建了一座高保真立体建筑群测试场：

| 场景功能区 | 结构组成与特征 | 算法验证价值 |
| :--- | :--- | :--- |
| **高层科技塔楼 A** | 高 6.5m 错落双层建筑、凹凸外立面、门廊 | 保证远距离垂直高程与密集大平面点云，消除高程退化 |
| **现代商业综合体 B** | 高 4.5m 砖红质感外立面、侧向防撞柱墩 | 提供多角度法向量约束与角点特征线 |
| **狭窄街道走廊 C** | L 型院落巷道 (Alleyway)、双侧墙壁 | 测试激光在狭长廊道几何特征受限时的鲁棒配准能力 |
| **立体缓坡台阶 D** | 12 度倾角缓坡 (Ramp)、阶梯平台 | 激发 6 轴 IMU 俯仰角动态变化，检验 3D 点云坡道投影与切片 |
| **立柱连廊 (Colonnade)** | 标准化圆柱阵列 (半径 0.22m，高 2.8m) | 激光雷达高曲率特征点 (Edge Points) 提取的黄金校验场 |
| **环形闭合干道 (Loop)** | 贯通整个街区的宽阔柏油主干道与中央环岛 | 回环检测 (Loop Closure) 与因子图位姿图优化的标准回路 |

---

## 3. 工程目录架构与模块化分层

```text
mobile_robot_station/
├── assets/                          # 核心资产库 (完全自包含，无外部软链接)
│   ├── models/turtlebot3/           # TurtleBot3 Burger 移动机器人物理与视觉模型
│   │   ├── meshes/                  # 自包含 STL 3D 几何网格 (base, wheels, sensors)
│   │   ├── scene.xml                # 机器人单体预览场景
│   │   └── turtlebot3_burger.xml    # 差速驱动动力学、IMU 与 3D 激光雷达挂载站点
│   └── scenes/                      # 仿真测试世界
│       └── urban_world.xml          # 专为 3D SLAM 打造的立体建筑群综合测试场
├── config/                          # 传感器参数与算法超参数 (YAML)
├── src/                             # 核心算法与业务逻辑模块化实现
│   ├── simulation/                  # 仿真驱动层
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
├── RULES.md                         # 项目工程规范与 AI 对话准则
├── .rules                           # 本地开发与协同规则配置
├── .gitignore                       # 规范忽略临时文件与编译缓存
└── README.md                        # 工业级系统文档
```

---

## 4. 快速启动指南

### 4.1 激活 Python 虚拟环境
本项目已通过系统专属虚拟环境验证：
```bash
source /home/yellowtown/Code/PythonProject/venv/bin/activate
```

### 4.2 启动小车与建筑群交互式仿真
运行启动脚本，即可拉起 MuJoCo 3D 视口，并在终端实时查看小车运行位姿与 IMU 遥测数据流：
```bash
python scripts/run_teleop_simulation.py
```

### 4.3 键盘遥控操作指令
小车采用标准双轮差速逆运动学模型，在打开的终端内直接键入控制：
- **`W` / `S`**：平滑增加 / 减小前进线速度 v (最大 0.22 m/s)；
- **`A` / `D`**：平滑增加 / 减小差速旋转角速度 w (最大 2.84 rad/s)；
- **`Space` (空格)**：紧急制动 (Emergency Stop)；
- **`Q` / `ESC`**：优雅退出仿真。

---

## 5. 五阶段技术演进路线图

- [x] **Stage 1.1：移动底座动力学与建筑群场景自包含交付**
  - [x] 完整拷贝 TurtleBot3 工业 CAD STL 网格，消除软链接；
  - [x] 调优接触阻尼与质量惯量比，杜绝数值振荡与 QACC 警告；
  - [x] 搭建带闭环回路的 3D 立体建筑群与街区测试场；
  - [x] 实现差速逆运动学与键盘非阻塞交互控制，代码模块化拆分。
- [ ] **Stage 1.2：CPU 极速 3D 激光雷达 Raycasting 引擎与 IMU 噪声模型**
  - [ ] 基于 MuJoCo 光线投射实现 16 线/32 线实时点云抓取；
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
- [ ] **Stage 5：Qt 6 QML 数字孪生移动监控站**
  - [ ] OpenGL/Qt3D 实时渲染点云与历史轨迹；
  - [ ] 航点导航与数字孪生遥测仪表盘。

---

## 6. 开源许可证
本项目遵循 Apache 2.0 开源许可证。
