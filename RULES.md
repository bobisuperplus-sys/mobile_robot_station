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
3. **强 CPU 与核显环境适配铁律 (Hardware Constraint)**：
   - 本项目宿主机环境为 Linux (Ubuntu 24.04) + 多核标压 CPU + 锐炬核显 (无独立强算力 GPU)；
   - 严禁提出或采用依赖重型 CUDA、大显存或需要独立 GPU 的技术选型 (如需大显存训练的 NeRF、3DGS 实时渲染器、NVIDIA Isaac Sim 等)；
   - 全面采用纯 CPU 并行优化友好、内存紧凑、利用 AVX 向量化与多线程加速的算法体系 (如 MuJoCo 多线程仿真、ikd-Tree、Eigen3、Sophus、Ceres 稀疏求解)。
4. **虚拟环境规范 (Python Environment)**：
   - 本项目固定的 Python 虚拟环境路径为: `source /home/yellowtown/Code/PythonProject/venv/bin/activate`；
   - 严禁在系统中随意创建新的冗余虚拟环境，所有依赖验证与脚本执行必须基于此环境。

---

## 2. 软件工程与模块化架构准则 (Modular Architecture)

1. **严禁单体大脚本 (Strict Modularization)**：
   - 严禁将运动学解算、物理仿真、通信网络、传感器驱动和交互控制混写在单个臃肿文件中；
   - 严格遵循功能分层架构：
     - `src/simulation/`: 专注于 MuJoCo 物理引擎接入、底盘动力学控制、多线激光雷达 Raycasting 与 IMU 数据流采集；
     - `src/core_math/`: 专注于空间几何、SE(3)/SO(3) 李群李代数切空间微扰求导、坐标系变换工具；
     - `src/slam/`: 专注于点云预处理、多线程 ICP/NDT 配准、ikd-Tree 动态数据结构维护、紧耦合里程计与位姿图优化；
     - `src/navigation/`: 专注于 3D 点云切片 2D Costmap 生成、A* 全局路径规划与 DWA 局部动态避障；
     - `src/gui/`: 专注于 Qt 6 QML 数字孪生上位机仪表盘与 3D 视口呈现；
     - `scripts/`: 仅作为轻量级组装入口，负责参数解析与子系统调度，不得堆砌底层算法实现。
2. **高内聚低耦合与接口规范**：
   - 模块间通过标准数据结构 (如 NumPy 数组、明确的类型注解 dataclass 或 Protobuf/JSON 消息) 交互，保持模块的可测试性与可替换性。

---

## 3. 资产管理与版本控制准则 (Asset & Git Integrity)

1. **完全自包含，坚决杜绝软链接 (Zero Symlinks)**：
   - 严禁在工程中使用任何外部软链接 (Symlink)；
   - 所有 3D CAD 模型、STL/OBJ 网格资产、材质纹理与场景文件，必须自包含存放在 `assets/` 对应的归档目录中，保证仓库克隆后立即可独立运行。
2. **仓库清洁度与临时文件审计**：
   - 提交前必须严格排查，杜绝将测试临时 XML、本地运行时日志 (如 MUJOCO_LOG.TXT)、Python 缓存 (__pycache__) 及构建二进制产物提交至 Git；
   - 保持 Git 提交记录语义化 (feat, fix, refactor, docs)。
