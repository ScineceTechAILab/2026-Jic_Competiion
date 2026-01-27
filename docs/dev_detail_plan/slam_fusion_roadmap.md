# 🧭 多传感器融合 SLAM 方案调研与路线图

> **文档状态**: 草稿 (Draft)  
> **创建日期**: 2025-01-24  
> **适用平台**: RDK X5 (ARM64) + 双目相机 + LiDAR (2D) + IMU  

## 1. 任务拆解 (Decomposition)

针对“双目视觉+LiDAR+IMU 融合 SLAM”这一复杂目标，我们将任务拆解为 **4 个核心阶段**，符合 V 型开发流程。

### 📌 阶段 I: 硬件与驱动基建 (Hardware & Driver)
**目标**: 确保数据“可用”、“同步”、“对齐”。
*   [ ] **硬件驱动节点化 (Driver Node Impl)**:
    *   **底盘**: 封装I2C通信，发布 `/wheel/odom` (nav_msgs/Odometry)，订阅 `/cmd_vel`。
    *   **IMU**: 发布 `/imu/data` (sensor_msgs/Imu)。
    *   **LiDAR**: 发布 `/scan` (sensor_msgs/LaserScan)。
    *   **Camera**: 发布 `/camera/image_raw` 及 `/camera/camera_info`。
*   [ ] **传感器频率验证**:
    *   IMU: 需 > 100Hz (推荐 200Hz+)，且波动 < 5%。
    *   LiDAR: 10Hz。
    *   Camera: 15-30Hz。
*   [ ] **时间同步 (Time Sync)**:
    *   验证各传感器 Topic 的 `header.stamp` 是否统一基于系统时间（或 PTP）。
    *   如果存在硬触发（Hardware Trigger），优先启用。
*   [ ] **坐标系标定 (Calibration)**:
    *   **内参**: 相机内参 (fx, fy, cx, cy, k1...)。
    *   **外参**: 
        *   `base_link` -> `imu_link` (6DOF)
        *   `base_link` -> `lidar_link`
        *   `base_link` -> `camera_link`
    *   *产出*: 更新 `urdf/robot.xacro` 中的静态 TF。

### 📌 阶段 II: 方案选型与验证 (Selection & Verification)
**目标**: 在 RDK X5 有限算力下找到精度与性能的平衡点。

| 方案 | 特性 | 优点 | 缺点/风险 | 适用场景 |
| :--- | :--- | :--- | :--- | :--- |
| **RTAB-Map** | 视觉+雷达+里程计 | **集成度高**，直接输出 2D 栅格地图对接 Nav2；支持断点续传。 | 视觉里程计 (VO) 较耗资源；纯视觉在白墙易丢失。 | **首选推荐** (通用性强) |
| **LVI-SAM** | VIO + LIO 紧耦合 | 精度极高，鲁棒性强。 | **不适用**: 强依赖 3D LiDAR (16线以上)；RDK 算力负载过高。 | **已排除** (因硬件限制) |
| **Cartographer** | 主要是雷达 | 2D 建图工业级标准，计算资源占用适中。 | 缺乏视觉回环检测，在长走廊易退化。 | **备选** (纯雷达方案) |

**推荐路线**: 
1.  **首选**: **RTAB-Map** (RGB-D + 2D LiDAR + IMU + Odom)，充分利用所有传感器。
2.  **备选**: 若视觉融合效果不佳，回退至 **Cartographer** (2D LiDAR + IMU)。

### 📌 阶段 III: 离线算法调优 (Offline Tuning)
**目标**: 在 PC 上验证算法，避免反复实车测试的低效。
*   [ ] **录制数据集 (Rosbag)**: 包含静止、直线、原地旋转、大回环场景。
*   [ ] **PC 端回放**: 在高性能 PC 上运行 SLAM 节点，调整参数（如特征点数量、关键帧间隔）。
*   [ ] **评估指标**: 轨迹闭合程度、地图清晰度、CPU 占用率。

### 📌 阶段 IV: 在线部署与 Nav2 对接 (Deployment)
**目标**: 实现实时定位与导航。
*   [ ] **RDK 部署**: 将调优后的参数迁移至板端。
*   [ ] **TogetheROS 优化**: 检查是否有地瓜机器人提供的硬件加速节点 (如使用 BPU 进行特征提取)。
*   [ ] **Nav2 联调**: 配置 `costmap_2d` 使用 SLAM 输出的 `/map` 和 `/odom`。

---

## 2. 详细执行 Checklist (Actionable Items)

### ✅ Day 1: 传感器体检
- [ ] 完成各硬件驱动的 ROS2 节点封装与独立运行测试。
- [ ] 检查 IMU 坐标系方向（通常 x-前, y-左, z-上）。
- [ ] 检查 TF 树是否完整 (`ros2 run tf2_tools view_frames`)。
- [ ] 录制一段 60秒 的 `rosbag`，包含所有传感器话题。

### ✅ Day 2: 标定与基础融合
- [ ] 运行 `robot_localization` (EKF)，融合 轮式里程计 + IMU，获得平滑的 `odom`。
- [ ] 标定相机-雷达外参（手动测量或使用 calibration 工具包）。

### ✅ Day 3: SLAM 算法跑通
- [ ] 安装 RTAB-Map ROS2: `sudo apt install ros-humble-rtabmap-ros`。
- [ ] 编写 launch 文件，订阅 `/odom`, `/scan`, `/camera/rgb`, `/camera/depth`。
- [ ] 仅使用雷达建图测试。
- [ ] 开启视觉 RGB-D 建图测试。

### ✅ Day 4: 性能优化
- [ ] 降低点云/图像分辨率以适配 RDK X5。
- [ ] 设置 `Reg/Strategy` 参数（0=Vis, 1=Icp, 2=Vis+Icp）。
