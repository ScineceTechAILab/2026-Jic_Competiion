# 🤖 【开源】AI 协作式 AGV (模块化AGV底盘移动操作臂系统)

> 🚀 基于地瓜机器人 RDK 的模块化移动操作臂系统，融合视觉感知、自主导航与跨模态交互技术。

<p align="center">
  <img src="https://img.shields.io/badge/Hardware-RDK_X5-orange?style=flat-square" />
  <img src="https://img.shields.io/badge/Robot-AGV_Manipulator-blue?style=flat-square" />
  <img src="https://img.shields.io/badge/ROS-2-green?style=flat-square" />
  <img src="https://img.shields.io/badge/License-MIT-lightgrey?style=flat-square" />
</p>

---

## 📖 目录

* [✨ 项目背景与简介](#-项目背景与简介)
* [🌟 核心方向与创新](#-核心方向与创新)
* [🛠 硬件与技术栈](#-硬件与技术栈)
* [� 项目规划与路线图](#-项目规划与路线图)
* [🧩 软件架构](#-软件架构)
* [� 安装与使用](#-安装与使用)
* [🤝 贡献](#-贡献)

---

## ✨ 项目背景与简介

本项目旨在研发一套**模块化 AGV 底盘移动操作臂系统（Mobile Manipulator System）**。

核心理念是将机械臂视作通用的“操作单元”，实现其在不同移动平台上的快速接入与协同联动。在此基础上，进一步引入**跨模态交互方式（BCI / MR / 语义控制）**，探索人机协同的差异化创新应用。

通俗来讲，即实现给拥有机械臂的小车赋予人工智能，使其能根据自然语言指令执行抓取和运输任务，进阶目标为实现多平台协同转运物品。

### 🎯 项目意义
*   **推动智能化服务落地**：解决物流、医疗、家庭服务等场景的劳动力不足问题。
*   **降低应用门槛**：模块化设计降低研发成本，促进普及。
*   **增强社会包容性**：为老年人、残障人士提供日常抓取和搬运帮助。
*   **助力教育**：作为教学平台，帮助学生实践机器人系统核心技术。

---

## 🌟 核心方向与创新

与传统单一平台的视觉导航小车或固定机械臂不同，本方案强调 **模块化 + 可扩展性**：

1.  **� 模块化设计**
    *   机械臂具备标准化接口。
    *   末端执行器（夹爪、吸盘等）支持快换。
    *   电路部分支持自动识别末端模块。

2.  **� 跨平台接入**
    *   操作臂可接入不同移动底盘（如轮式小车、AGV 或桌面实验平台）。
    *   实现“换平台/换任务”的快速切换。

3.  **🧠 跨模态交互**
    *   用户通过 **脑机接口 (BCI)** 或 **混合现实 (MR)** 眼镜发出指令。
    *   MR 提供空间增强反馈，实现更自然的人机交互。

---

## 🛠 硬件与技术栈

### 硬件平台
*   **核心计算单元**：地瓜机器人 RDK X5 / RDK X3
*   **移动底盘**：自主研发或开源 AGV 底盘
*   **机械臂**：2~3 自由度轻量级机械臂
*   **传感器**：摄像头 (视觉感知)、激光雷达 (LDS-50C / RPLidar)、编码器

### 技术栈
*   **操作系统**：Ubuntu 22.04
*   **中间件**：ROS 2 (Robot Operating System)
*   **算法**：
    *   **SLAM & 导航**：Nav2, Cartographer / AMCL
    *   **视觉感知**：YOLO-tiny / MobileNet (目标检测), OpenCV (图像处理)
    *   **运动控制**：PID 控制, 逆运动学 (IK)
*   **交互**：Unity (MR开发), BCI SDK

---

## � 项目规划与路线图

项目分为两个核心阶段：**核心 Demo (80%)** 和 **创新探索 (20%)**。

### 📌 阶段 1：移动平台路径规划 (3周)

#### 线性规划

- [x] 编写验证底盘驱动的代码。
    - [x] 底盘的定性基本运动形式代码编写验证
    - [x] 底盘的定量基本运动形式代码编写验证
        - [x] IMU 驱动代码的编写验证
        - [x] 里程计代码的编写验证
        - [x] web 测试网页
    - [x] 底盘测试代码(基于IMU和里程计双校验)
- [x] 底盘运动学建模与驱动控制验证。
- [ ] 调研并验证适合双目视觉+LiDAR+IMU融合的 SLAM 方案 (如 RTAB-Map, LVI-SAM)。
- [ ] 初步配置 Nav2 导航栈以对接 SLAM。
- [ ] 激光雷达/视觉简易避障。
- **输出**：底盘可从起点自主移动到目标区域并构建环境地图。

#### 反馈规划

- [ ] 重新校验现有实现和原有架构设计

### 📌 阶段 2：机械臂操纵 (3周)

#### 线性规划

- [ ] 搭建 MCU 与 RDK 的通讯接口 (CAN/UART)。
- [ ] 完成关节 PID 控制和角度反馈验证。
- [ ] 简单运动测试 (预设动作)。
- **输出**：机械臂可接受指令并完成重复性动作。

#### 反馈规划

- [ ] 重新校验现有实现和原有架构设计

### 📌 阶段 3：机械臂视觉抓取 (4周)
- [ ] 接入摄像头 + 感知模块 (OpenCV/轻量 AI)。
- [ ] 完成像素坐标 → 机械臂坐标系的标定。
- [ ] 实现简单目标检测 (颜色/形状)。
- [ ] 机械臂执行抓取 (吸盘/夹爪)。
- **输出**：能够识别并抓取静态桌面上的目标。

### � 阶段 4：机械臂 + 移动平台协作搬运 (3周)
- [ ] 将视觉抓取与路径规划集成。
- [ ] 流程：底盘移动到目标点 → 机械臂抓取 → 底盘运送 → 机械臂放置。
- [ ] 定义任务管理器的流程控制。
- **输出**：系统可完成搬运流程的 Demo (单一物体搬运)。

### 📌 阶段 5：机械臂接口模块化 (2周)
- [ ] 设计模块 ID 电路 (电阻 ID / 插针检测)。
- [ ] MCU 实现自动识别与参数上报。
- [ ] RDK 任务管理器自动切换抓取策略。
- **输出**：可在不改代码的情况下更换夹爪/吸盘等末端模块。

### 📌 阶段 6：机械臂操作多模态化 (3周，扩展)
- [ ] 接入 MR 眼镜 (Unity/SDK) 与 HRI 接口。
- [ ] 接入 BCI 信号或语义解析 SDK。
- [ ] 定义多模态输入与任务映射规则。
- **输出**：可通过 MR/BCI/语音等方式下达任务指令。

---

## 🧩 软件架构

```mermaid
graph TD
    User[用户 (MR/BCI/语音)] -->|指令| RDK[RDK 主控]
    
    subgraph "RDK 主控 (ROS 2)"
        Task[任务管理器]
        SLAM[SLAM (RTAB-Map/Cartographer)]
        Nav[导航与规划 (Nav2)]
        Vision[视觉感知 (YOLO/OpenCV)]
        ArmCtrl[机械臂控制]
        BaseCtrl[底盘控制]
    end
    
    RDK -->|CAN/UART| MCU[底层 MCU]
    
    subgraph "硬件执行"
        MCU --> Motor_Base[底盘电机]
        MCU --> Motor_Arm[机械臂舵机]
        MCU --> End_Effector[末端执行器 (可换)]
    end
    
    Camera -->|图像流| Vision
    Camera -->|双目/深度| SLAM
    Lidar -->|2D 扫描| SLAM
    IMU -->|惯导数据| SLAM
    
    SLAM -->|定位/地图| Nav
    
    Task --> Nav
    Task --> Vision
    Task --> ArmCtrl
    
    Nav --> BaseCtrl
```

---

## 📦 安装与使用

### 1. 克隆仓库
```bash
git clone https://github.com/YourTeam/ai-agv-manipulator.git
cd ai-agv-manipulator
```

### 2. 安装依赖
```bash
# 安装 ROS 2 依赖
rosdep install --from-paths src --ignore-src -r -y

# 安装 Python 依赖
pip3 install -r requirements.txt
```

### 3. 编译工作空间
```bash
# 使用提供的构建脚本（自动指定输出目录为 cache/）
chmod +x build.sh
./build.sh

# Source 环境变量
source cache/install/setup.bash
```

> **注意**: 为保持项目根目录整洁，所有构建产物（build, install, log）均存放在 `cache/` 目录下。

### 4. 运行
```bash
# 启动底盘驱动
ros2 launch chassis_driver chassis.launch.py

# 启动导航
ros2 launch nav2_bringup navigation_launch.py

# 启动任务演示
ros2 run task_manager demo_mission
```

---

## 🤝 贡献

欢迎参与本项目！

### 👥 团队分工

*   **Team Leader (统筹与集成)**: 负责系统调度、进度管理与集成冲刺。
*   **算法开发 (Algorithm)**:
    *   **B 成员**: 负责视觉检测（YOLO-tiny/MobileNet）与目标识别。
    *   **C 成员**: 负责坐标系标定、路径规划与 SLAM。
*   **硬件开发 (Hardware)**:
    *   **A/B 成员**: 负责电路设计（驱动板、电源干扰处理、模块识别电路）。
    *   **D 成员**: 负责机械设计（机械臂、底盘结构、3D 打印）。

### 📄 文档管理

*   **开发日记 (`docs/diary/`)**: 记录每日开发进度、Bug 修复与调试心得。
*   **技术日志 (`docs/dev_logs/`)**: 沉淀技术方案、SOP (标准作业程序) 与核心算法原理。

---

## 📄 许可证

本项目采用 [MIT License](LICENSE) 许可证。
