# RULES: mobile_robot_station 项目工程开发与 AI 交互行为准则

本文档既是 `mobile_robot_station` 项目全生命周期开发规范，也是与用户在该项目中所有对话与协同中 AI 必须无条件严格遵守的核心规则。

---

## 1. AI 交互与对话行为准则 (AI Agent Core Directives)

1. **语言与注释要求 (Language Requirement)**：
   - AI 的所有对话回复、技术解释、规划文档与代码修改说明，一律使用**简体中文 (Simplified Chinese)**；
   - 源码中的所有文件头说明、函数 Docstring、行内注释与配置说明，一律使用**简体中文**编写。
2. **文本风格与严禁 Emoji (Strictly Forbid Emojis)**：
   - 严禁在任何对话回复、Markdown 文档、代码文件、注释文本、终端打印信息中使用任何 Emoji 表情符号；
   - 保持严谨、专业、克制与客观的技术工程风格，禁止多余的情绪化修辞。
3. **强 CPU 与轻量化计算适配原则 (Hardware & Compute Principles)**：
   - 面向通用标准平台与多核 CPU 环境，彻底规避依赖重型 CUDA、大显存或必须依赖独立高端 GPU 的框架；
   - 优先采用纯 CPU 并行优化友好、内存紧凑、利用 AVX 向量化与多线程加速的高性能算法体系 (如 MuJoCo 多线程仿真、ikd-Tree、Eigen3、Sophus、Ceres 稀疏求解器)。

---

## 2. 软件工程与模块化架构准则 (Modular Architecture)

1. **严禁单体大脚本 (Strict Modularization)**：
   - 严禁将运动学解算、物理仿真、通信网络、传感器驱动和交互控制混写在单个臃肿文件中；
   - 严格遵循功能分层架构：
     - `src/simulation/`: 专注于物理引擎接入、底盘动力学控制、多线激光雷达 Raycasting 与 IMU 遥测数据流采集；
     - `src/core_math/`: 专注于空间几何、SE(3)/SO(3) 李群李代数切空间微扰求导、坐标系变换工具；
     - `src/slam/`: 专注于点云预处理、多线程 ICP/NDT 配准、ikd-Tree 动态数据结构维护、紧耦合里程计与位姿图优化；
     - `src/navigation/`: 专注于 3D 点云切片 2D Costmap 生成、A* 全局路径规划与 DWA 局部动态避障；
     - `src/gui/`: 专注于 Qt 6 QML 数字孪生上位机仪表盘与 3D 视口呈现；
     - `scripts/`: 仅作为轻量级组装入口，负责参数解析与子系统调度，不得堆砌底层算法实现。
2. **高内聚低耦合与接口规范**：
   - 模块间通过标准数据结构 (如 NumPy 数组、明确的类型注解 dataclass 或标准消息格式) 交互，保持各个子模块独立可测试与可替换。
