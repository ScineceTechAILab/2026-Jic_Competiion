**AGENT AGV**

BCI × 具身智能 × MR 四层闭环系统

意图接入 · Agent调度 · 具身执行 · MR空间反馈

工程规划文档  v4.0  |  独立开发  |  周期：12个月  |  基于仓库实际进度更新

# **0  当前进度总览（基于仓库实际状态）**

**本节根据仓库代码实际状态更新。绿色=已完成，黄色=进行中，橙色=待开始**

## **0.1  已完成模块**

| **模块** | **文件/路径** | **完成情况** |
| --- | --- | --- |
| 底盘驱动 | src/support/driver/chassis_driver.py | 完整实现：差速运动学、I2C通信、编码器反馈、速度修正 |
| IMU驱动 | src/support/driver/imu_driver.py | BNO055 已调通，Bus 5，地址 0x29，四元数+角速度+加速度 |
| 摄像头驱动 | src/support/driver/camera_driver.py | USB摄像头已调通，支持 JPEG 流 |
| 激光雷达串口 | tests/lidar/lidar_connect.py | RPLIDAR S1 串口连接测试通过，/dev/ttyS1，256000 波特 |
| 配置加载系统 | src/support/config_loader.py | YAML配置加载，支持底盘/IMU参数 |
| 日志系统 | src/support/log/log.py | RotatingFileHandler + 控制台双输出，完整 |
| Web调试面板 | web/api/main.py + web/public/ | FastAPI后端 + SPA前端，含底盘控制/IMU/雷达/摄像头 |
| 底盘电机调试 | tests/chassis/ | 单电机、旋转、平移测试脚本均已完成 |
| 项目分层架构 | src/{support,execution,functional,decision,application}/ | 五层目录结构已建立，接口文件已创建 |

## **0.2  待实现模块**

| **模块** | **文件/路径** | **当前状态** | **对应阶段** |
| --- | --- | --- | --- |
| 激光雷达ROS2驱动 | src/functional/slam_module/tof_slam.py | 空模板，lidar_driver 为 Mock | 第一阶段 |
| URDF建模 | src/support/simulation/ | launch模板，无URDF | 补课项 |
| SLAM建图 | src/functional/slam_module/ | 空模板，Nav2未配置 | 第一阶段 |
| 机械臂控制 | src/execution/arm_control/flexiv_adapter.py | 空模板，硬件未接入 | 第二阶段 |
| 视觉感知 | src/functional/visual_perception/ | calibrator/detector 未实现 | 第三阶段 |
| 路径规划 | src/decision/path_planning/nav2_adapter.py | 空模板，Nav2未接入 | 第一阶段 |
| 行为树 | src/decision/behavior_tree/ | 空模板 | 第三阶段 |
| 任务调度 | src/application/task_planner/task_scheduler.py | 空模板 | 第三阶段 |
| BCI接入 | src/application/bci_mr_interaction/bci_interface.py | 空模板 | 第四阶段 |
| MR反馈层 | src/application/bci_mr_interaction/mr_interface.py | 空模板（新增核心模块） | 第四阶段 |

## **0.3  实际硬件参数**

| **硬件** | **实际参数** | **来源** |
| --- | --- | --- |
| 主控 | RDK X5（地瓜机器人） | package.xml / README |
| 底盘电机板 | I2C Bus 5，地址 0x26，4路电机 | config/chasis_params.yaml |
| 底盘参数 | 轮距 160mm，轮径 65mm，减速比 30，磁环线数 11 | config/chasis_params.yaml |
| 电机映射 | 左轮 M3，右轮 M2，方向修正各 1.5x | config/chasis_params.yaml |
| IMU | BNO055，I2C Bus 5，地址 0x29 | config/imu_params.yaml |
| 激光雷达 | RPLIDAR S1，串口 /dev/ttyS1，波特率 256000 | tests/lidar/lidar_connect.py |
| 摄像头 | USB摄像头，/dev/video0，640x480 | tests/camera/ |
| MR眼镜 | Meta Quest 3（视频透传MR，OpenXR/Unity XR Toolkit） | 新增硬件 |

# **1  项目定位**

## **1.1  核心命题：四层闭环系统**

本项目不是简单的「BCI触发机器人「，而是一套完整的四层闭合回路人机协同系统。系统的核心命题在于：将操作者的脑电意图、智能体任务调度、具身机器人执行、混合现实空间反馈四个环节串联成一个完整的认知-执行闭环，使人成为回路中的有效组成部分。

**【L1 意图接入层】**  BCI / EEG → 专注度阈值 → START 指令

↓  START

**【L2 Agent 调度层】**  任务状态机 → 任务分解 → cmd_vel / 臂控指令

↓  cmd_vel

**【L3 具身执行层】**  Nav + 机械臂 + 视觉 → 抓取搬运 → Sensor data

↓  Sensor data

**【L4 MR 反馈层】**  Meta Quest 3 → 路径预览 + 任务状态 → Visual feedback → L1

*闭环说明：操作者感知 MR 反馈 → 调整注意力状态 → BCI 触发下一轮指令 → 循环*

## **1.2  MR 反馈层的战略意义**

在没有 MR 反馈层的系统中，操作者只能凭借对机器人的观察来判断任务执行状态，存在信息延迟和认知负担。MR 反馈层的引入将系统从「开环触发「升级为「闭环协同「：

- 路径预览：在操作者视野中叠加机器人规划路径，使操作者在任务开始前即可预判执行轨迹

- 任务状态：实时显示当前状态机节点（导航中/抓取中/放置中），降低操作者认知负担

- 意图校正：操作者根据 MR 反馈调整注意力集中程度，形成意图的动态调节

## **1.3  差异化对比**

| **维度** | **传统视觉导航小车** | **BCI驱动机器人（已有研究）** | **本项目 AGENT AGV** |
| --- | --- | --- | --- |
| 控制输入 | 键盘/手柄 | EEG离散分类指令 | EEG专注度连续阈值 |
| 反馈方式 | 屏幕监控 | 无空间反馈 | MR空间叠加反馈（Quest 3） |
| 闭环特性 | 开环 | 开环 | 四层认知-执行闭环 |
| 末端扩展 | 无 | 无 | 快换接口+模块自动识别 |
| 移动+操作 | 分离 | 分离 | 一体化移动操作臂 |
| 技术融合深度 | 单栈 | 双栈（BCI+机器人） | 四栈（BCI+具身+MR+Agent） |

# **2  系统架构**

## **2.1  四层闭环软件架构**

| **层级** | **模块** | **核心职责** | **关键接口** | **完成度** |
| --- | --- | --- | --- | --- |
| L1 意图接入层 | bci_interface.py | EEG专注度采集，阈值检测，触发START/PAUSE/ABORT | /bci/attention_state /bci/task_trigger | 空模板 |
| L2 Agent调度层 | task_scheduler.py behavior_tree_node.py | 接收BCI触发，任务分解，状态机管理，异常处理 | /agent/task_command ROS2 Action | 空模板 |
| L3 具身执行层 | chassis_driver.py（已完成） flexiv_adapter.py detector.py | 底盘导航，机械臂控制，视觉感知，末端识别 | /cmd_vel /odom /scan /arm/joint_states | 底盘完成 |
| L4 MR反馈层 | mr_interface.py Unity XR Toolkit + OpenXR | 接收传感器数据，渲染路径/状态叠加层，回传视觉反馈 | WebSocket/ROS2 bridge Unity ↔ RDK X5 | 空模板（新增） |

## **2.2  MR 通信架构**

Meta Quest 3 运行 Unity 应用，通过 WebSocket 或 ROS2 Bridge 与 RDK X5 通信，接收以下数据并渲染到 AR 视野：

- 机器人当前位姿（来自 /odom）→ 在真实空间中渲染机器人虚拟轮廓

- 规划路径（来自 Nav2 /plan）→ 在地面叠加路径预览线

- 任务状态字符串（来自 task_scheduler）→ 在视野角落显示状态 HUD

- 目标物位置（来自 /vision/target_pose）→ 在目标物上叠加高亮标注

## **2.3  各子系统说明**

### **2.3.1  底盘子系统（已完成）**

ChassisDriver 封装 I2C 4路电机驱动板（地址 0x26），提供 move/rotate/wheel_speed/battery_voltage 标准接口。下一步封装为 ROS2 节点，发布 /odom，订阅 /cmd_vel，接入 Nav2。

### **2.3.2  传感器子系统（已完成）**

IMU（BNO055）已调通，输出四元数姿态。摄像头 USB 流已调通。RPLIDAR S1 串口通信验证通过，待封装为 ROS2 LaserScan 话题。注意：当前 lidar_driver.py 为 Mock 数据，M0 阶段优先替换为真实驱动。

### **2.3.3  Web 调试面板（已完成）**

FastAPI + SPA 前端，支持底盘实时控制、IMU 监控、雷达可视化、摄像头流。贯穿全开发周期作为调试工具。

### **2.3.4  SLAM 与导航（待开发）**

SLAM Toolbox 室内建图 + Nav2 路径规划（A* + DWA）。依赖激光雷达 ROS2 驱动和底盘 ROS2 节点先完成。

### **2.3.5  机械臂子系统（待开发）**

当前 flexiv_adapter.py 为空模板。需先确认机械臂型号，选择 MoveIt2 或自定义 PID 控制方案。末端执行器通过电阻 ID 电路自动识别夹爪/吸盘类型。

### **2.3.6  BCI 接入子系统（待开发）**

接入非侵入式 EEG 设备，采用专注度连续阈值触发策略（≥70/100 持续 2 秒 → START），键盘保留为备用触发方式，确保 Demo 可靠性。

### **2.3.7  MR 反馈子系统（待开发，新增核心模块）**

基于 Meta Quest 3 + Unity SDK 开发 AR 覆盖层，订阅 ROS2 话题数据，通过 WebSocket Bridge 推送至 Unity 端实时渲染。实现路径预览（Path Preview）和任务状态 HUD（Task Status），完成闭环最后一环。

# **3  硬件选型**

| **模块** | **实际选型** | **状态** | **备注** |
| --- | --- | --- | --- |
| 主控计算机 | RDK X5（地瓜机器人） | 已到位 | 运行 Ubuntu，支持 ROS2 |
| 底盘电机驱动 | I2C 4路电机板，地址 0x26 | 已调通 | 差速驱动，编码器反馈 |
| IMU | BNO055，I2C Bus 5，0x29 | 已调通 | 四元数姿态输出 |
| 激光雷达 | RPLIDAR S1，/dev/ttyS1 | 串口通过，ROS2待封装 | 使用 rplidar_ros 社区包适配 |
| 摄像头 | USB摄像头，/dev/video0 | 已调通 | 640x480，MJPEG 流 |
| 机械臂 | 待确认型号 | 待确认 | 需明确后制定控制方案 |
| 末端执行器 | 夹爪/吸盘（快换接口） | 规划中 | 电阻 ID 电路识别模块类型 |
| MR眼镜 | Meta Quest 3 | 新增（待采购） | Unity SDK，WebSocket通信，视频透传MR，空间锚定精度高 |
| EEG 设备 | NeuroSky MindWave / Emotiv Insight | 第四阶段采购 | Python SDK，专注度输出 |

# **4  软件技术栈**

| **类别** | **技术/框架** | **状态** | **用途** |
| --- | --- | --- | --- |
| 底层驱动 | smbus2 / adafruit-bno055 | 已在用 | I2C通信，传感器读取 |
| Web框架 | FastAPI + uvicorn | 已在用 | 调试 API 与前端面板 |
| 视觉库 | OpenCV | 已引入 | 摄像头帧处理 |
| 机器人中间件 | ROS2 Humble | 待配置 | 模块间通信 Topic/Action/Service |
| 导航 | Nav2（SLAM Toolbox + DWA） | 待接入 | 自主建图与路径规划 |
| 操作臂控制 | MoveIt2 / 自定义 IK | 待定 | 运动学求解，轨迹规划 |
| 目标检测 | YOLO-tiny / MobileNetV3 | 待实现 | 物体识别，像素坐标转换 |
| MR开发 | Unity + Meta Quest 3 SDK | 待接入（新增） | AR叠加层，路径/状态渲染 |
| ROS-Unity桥 | ROS2 For Unity / WebSocket | 待配置（新增） | RDK X5 与 Quest 3 数据互通 |
| BCI SDK | NeuroSky / Emotiv Python SDK | 待接入 | EEG专注度阈值触发 |
| 仿真 | Gazebo Ignition | 模板已建 | URDF验证（补课项） |
| 上层语言 | Python 3.10 + C++17 | Python已在用 | Python调度，C++实时控制 |

# **5  通信接口规范**

## **5.1  已实现接口（FastAPI REST，调试用）**

| **接口** | **方向** | **说明** |
| --- | --- | --- |
| POST /api/control | 外部 -> 底盘 | move_forward/backward/rotate_cw/ccw/stop |
| GET /api/battery | 底盘 -> 前端 | 电池电压（V） |
| GET /api/chassis/speed | 底盘 -> 前端 | 左右轮实时速度（m/s） |
| GET /api/imu | IMU -> 前端 | 四元数、角速度、加速度 |
| GET /api/lidar | 雷达 -> 前端 | 激光扫描数据（当前为 Mock） |
| GET /api/camera/stream | 摄像头 -> 前端 | MJPEG 视频流 |
| POST /api/config | 前端 -> 系统 | 速度修正系数更新 |

## **5.2  规划接口（ROS2，正式集成）**

| **接口** | **方向** | **类型** | **数据内容** |
| --- | --- | --- | --- |
| /bci/attention_state | BCI -> Agent | Topic (Float32) | 专注度 0~100，10Hz |
| /bci/task_trigger | BCI -> Agent | Topic (String) | START / PAUSE / ABORT |
| /agent/task_command | Agent -> 底盘/臂 | Action | 任务类型+目标参数 |
| /cmd_vel | Agent -> 底盘 | Topic (Twist) | 线速度+角速度 |
| /odom | 底盘 -> Agent/MR | Topic (Odometry) | 里程计位姿 |
| /scan | 雷达 -> Nav2 | Topic (LaserScan) | 激光扫描数据 |
| /vision/target_pose | 视觉 -> Agent | Topic (PoseStamped) | 目标物位姿 |
| /arm/joint_states | 臂 -> Agent | Topic (JointState) | 关节角度反馈 |
| /mr/robot_state | Agent -> MR | WebSocket / Topic | 机器人位姿+任务状态+路径 |
| /mr/feedback | MR -> Agent | WebSocket / Topic | 操作者确认/中止手势（扩展） |

# **6  更新后的时间规划（v3，含 MR 闭环）**

**当前状态：底盘驱动层已完成，处于第零阶段（URDF补课）起点。MR 反馈层升级为核心开发目标，不再是可选扩展。**

## **第零阶段（补课）：URDF 建模与 ROS2 基础配置（第 1~2 周）**

目标：将已完成实体硬件反向建模为 URDF，封装 ROS2 底层节点。

- 测量底盘物理参数（轮距160mm、轮径65mm），建立 robot.urdf.xacro

- 在 RViz 中验证 URDF 模型与实体外观一致

- 封装 ChassisDriver 为 ROS2 节点，发布 /odom，订阅 /cmd_vel

- 封装 RPLIDAR S1 为 ROS2 节点，发布 /scan（替换 Mock 驱动）

**交付物：**RViz 中车体+雷达数据正常显示，/odom 与 /scan 话题正常发布

## **第一阶段：SLAM 建图与自主导航（第 3~6 周）**

目标：实现室内自主建图与点到点导航。

- 配置 SLAM Toolbox，真实场景建图，保存地图文件

- 配置 Nav2（A* + DWA），实现点到点自主导航

- 实现动态障碍物避障（基于激光雷达代价地图）

- obstacle_filter.py 接入 /scan 话题，完成避障模块

**交付物：**底盘在真实室内场景自主导航，避障成功，精度 ±10cm

## **第二阶段：机械臂接入与基础控制（第 7~11 周）**

目标：确认机械臂型号，实现基础运动控制与手眼标定。

- 确认机械臂型号，选择 MoveIt2 或自定义 IK 控制方案

- 实现关节角度控制与末端位置控制（flexiv_adapter.py 或 arm_driver.py）

- 完成手眼标定，calibrator.py 实现像素坐标 → 机械臂坐标系转换

- 设计末端快换接口与模块 ID 电路（电阻分压识别夹爪/吸盘）

**交付物：**机械臂完成预设动作，手眼标定完成，末端模块可自动识别

## **第三阶段：视觉抓取（第 12~16 周）**

目标：基于视觉实现静态目标识别与抓取。

- detector.py：YOLO-tiny / MobileNetV3 识别颜色+形状

- 发布 /vision/target_pose，输出目标物在机械臂坐标系中的位姿

- 机械臂抓取闭环：视觉定位 → IK → 关节轨迹 → 夹爪控制

- 桌面静态目标识别并抓取，成功率 ≥ 70%

**交付物：**静态目标识别并抓取，成功率 ≥ 70%

## **第四阶段：系统集成、BCI 接入与 MR 闭环（第 17~36 周）**

目标：打通四层闭环全链路，这是项目最核心的集成阶段。

- 第 17~24 周：Agent 任务调度层（task_scheduler.py + behavior_tree_node.py），底盘+机械臂联合调试，完成无 BCI 搬运 Demo

- 第 25~30 周：EEG 设备 SDK 调试，bci_interface.py 实现专注度阈值检测，接入 task_scheduler

- 第 31~36 周：Meta Quest 3 + Unity 开发 MR 反馈应用，ROS2 Bridge 打通数据链路，实现路径预览与任务状态 HUD

- 第 36 周：四层闭环联调——BCI 触发 → Agent 调度 → 机器人执行 → MR 反馈 → 操作者调整注意力 → 闭环

**交付物：**M5 里程碑：四层闭环 Demo，BCI 触发，MR 实时反馈，端到端成功率 ≥ 70%

## **第五阶段：优化与最终演示（第 37~48 周）**

目标：系统打磨，完成最终演示准备。

- 性能优化：导航精度、抓取鲁棒性、BCI 信噪比、MR 渲染延迟

- MR 体验优化：AR 叠加层视觉效果、操作者认知负担评估

- 最终集成测试；Demo 场景布置；演示视频制作；项目文档整理

**交付物：**M6 里程碑：完整四层闭环 Demo，可面向答辩稳定演示

# **7  MVP 与里程碑定义（v3）**

| **里程碑** | **时间节点** | **验收标准** | **状态** | **优先级** |
| --- | --- | --- | --- | --- |
| M0 URDF + ROS2底盘节点 | 第2周末 | URDF在RViz正常，/odom与/scan正常发布 | TODO | P0 |
| M1 SLAM + 自主导航 | 第6周末 | 室内自主建图，Nav2导航成功，避障有效 | TODO | P0 |
| M2 机械臂基础控制 | 第11周末 | 机械臂完成预设动作，手眼标定完成 | TODO | P0 |
| M3 视觉抓取 | 第16周末 | 静态目标识别并抓取，成功率 ≥ 70% | TODO | P0 |
| M4 搬运全流程Demo | 第24周末 | 完整搬运，手动触发，成功率 ≥ 80% | TODO | P0 |
| M5 四层闭环联调 | 第36周末 | BCI触发+MR实时反馈，端到端成功率 ≥ 70% | TODO | P1 |
| M6 最终演示就绪 | 第48周末 | 稳定演示，Demo视频完成，文档整理完毕 | TODO | P1 |
| [已完成] 底盘驱动 | 已完成 | ChassisDriver完整实现，Web面板可控 | DONE | P0 |
| [已完成] 传感器驱动 | 已完成 | IMU/摄像头已调通，雷达串口验证 | DONE | P0 |

# **8  风险管理（v3，含 MR 风险）**

| **风险** | **概率** | **影响** | **应对预案** |
| --- | --- | --- | --- |
| 机械臂型号/控制方案不明确 | 高 | 高 | 立即确认型号；不影响其他模块并行推进 |
| Meta Quest 3 SDK 适配困难 | 中 | 高 | 提前调研 Unity XR Toolkit + OpenXR 文档；备用方案：手机 ARCore 替代实现基础 AR 叠加 |
| ROS2 Bridge 与 Unity 通信延迟高 | 中 | 中 | 使用 WebSocket 直连优化延迟；MR 刷新率目标 ≥ 10Hz |
| EEG 信号质量差，触发不稳定 | 高 | 中 | 设置 2s 判定窗口；键盘作备用触发；不阻塞前三阶段 |
| 机械臂抓取成功率低 | 高 | 高 | 退化为吸盘方案；限制目标物形状；调整手眼标定精度 |
| SLAM 建图漂移 | 中 | 高 | 室内添加 AR 标记辅助；预存已知地图（仅定位模式） |
| 四层联调时序复杂 | 高 | 高 | 严格按接口契约开发；每层独立测试后再集成；频繁集成检查 |
| 单人开发，单点瓶颈 | 高 | 高 | MVP 优先；M4（无BCI搬运Demo）已是完整成果 |
| lidar_driver 目前为 Mock | 已知 | 中 | M0 阶段优先替换真实 ROS2 驱动，废弃 Mock |

# **9  创新点阐述**

## **9.1  四层认知-执行闭环（核心创新）**

本项目的最大创新在于将 BCI、Agent、具身智能、MR 四个技术栈整合为一个完整的闭合回路，而非四个独立功能的堆叠。闭环的关键在于：MR 反馈层的输出（空间可视化信息）作为操作者认知系统的输入，影响其注意力状态，进而通过 BCI 层形成下一轮控制信号。这一设计使人成为回路中的有效调节环节，是传统机器人遥操作系统所不具备的特性。

## **9.2  EEG 阈值触发的工程化设计决策**

有意选择连续专注度阈值触发而非离散 EEG 分类，从架构层面规避了个体差异导致的分类器泛化失败。这一决策使系统具备跨用户稳定性，是工程可行性优先于技术炫耀的务实选择，也体现了对 BCI 在现阶段技术成熟度的清醒认知。

## **9.3  模块化末端自动识别**

末端执行器通过电阻 ID 电路实现自动类型识别，Agent 层无需手动配置即可切换夹爪/吸盘抓取策略。这一设计体现了「可重构「理念，使同一套系统可快速适配不同任务场景。

## **9.4  分层可调试工程架构**

从驱动层开始即建立了完善的 Web 调试面板与独立测试脚本，每层模块可独立验证（底盘、IMU、摄像头各有测试脚本）。这种工程设计降低了单人开发的集成风险，也为后续开源社区复现提供了友好的调试入口。

# **10  未来扩展方向**

- 多模态意图融合：EEG + EMG 肌电 + 眼动追踪融合，提升意图识别鲁棒性

- MR 手势交互：在 MR 层增加手势输入，作为 BCI 信号的补充控制方式

- 多平台协同：Agent 层扩展为多 Agent，协调多台机器人协同搬运

- 语义控制：LLM 驱动自然语言任务下达，补充 BCI 触发方式

- 更高自由度机械臂：接入 6-DOF 臂，扩展操作复杂度

AGENT AGV  v4.0  |  BCI × 具身智能 × MR 四层闭环系统  |  基于仓库实际进度更新