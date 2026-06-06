# AGENT AGV — ROS2 全面命名规范

> 适用范围：本项目所有 ROS2 节点、话题、服务、动作、参数、TF 坐标系、文件与目录。
> 基于 REP-144 / REP-149 / ROS2 社区惯例，结合四层闭环架构定制。

---

## 1 总则

| 规则 | 说明 |
|------|------|
| 字符集 | 小写字母 `a-z`、数字 `0-9`、下划线 `_` |
| 禁止字符 | 大写字母、连字符 `-`、空格、中文 |
| 起始字符 | 必须以字母开头，不能以数字或下划线开头 |
| 命名空间分隔 | 用 `/` 划分层级（话题/服务/动作） |
| 长度建议 | 单段名称 ≤ 30 字符，完整路径 ≤ 60 字符 |

---

## 2 ROS2 包（Package）

格式：`agent_agv_<功能域>`

```
agent_agv_bringup        # 启动文件、launch
agent_agv_description    # URDF / meshes
agent_agv_driver         # 底盘/IMU/雷达底层驱动节点
agent_agv_navigation     # SLAM + Nav2 配置
agent_agv_perception     # 视觉感知、目标检测
agent_agv_arm            # 机械臂控制
agent_agv_agent          # 行为树 + 任务调度（L2）
agent_agv_bci            # BCI 意图接入（L1）
agent_agv_mr             # MR 反馈层（L4）
agent_agv_interfaces     # 自定义 msg/srv/action 定义
```

---

## 3 节点（Node）

格式：`<功能>_node`，`snake_case`

```
chassis_driver_node       # 底盘驱动
imu_driver_node           # IMU 驱动
lidar_driver_node         # 激光雷达驱动
camera_driver_node        # 摄像头驱动
slam_node                 # SLAM 建图
nav2_adapter_node         # Nav2 导航适配
arm_driver_node           # 机械臂驱动
arm_ik_node               # 逆运动学求解
detector_node             # 目标检测
calibrator_node           # 手眼标定
task_scheduler_node       # 任务调度（L2）
behavior_tree_node        # 行为树执行
bci_interface_node        # BCI 接入（L1）
mr_bridge_node            # MR WebSocket 桥（L4）
end_effector_id_node      # 末端模块识别
web_debug_node            # Web 调试面板桥接
```

---

## 4 话题（Topic）

格式：`/<命名空间>/<数据描述>`

### 4.1 按四层架构分组

**L3 具身执行层（基础传感/执行）**

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | Agent → 底盘 | 速度指令 |
| `/odom` | `nav_msgs/Odometry` | 底盘 → Nav2/Agent | 里程计 |
| `/scan` | `sensor_msgs/LaserScan` | 雷达 → Nav2 | 激光扫描 |
| `/imu/data` | `sensor_msgs/Imu` | IMU → 融合 | IMU 原始数据 |
| `/camera/image_raw` | `sensor_msgs/Image` | 摄像头 → 感知 | 原始图像 |
| `/camera/image_compressed` | `sensor_msgs/CompressedImage` | 摄像头 → 调试 | 压缩图像流 |

**L3 机械臂**

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/arm/joint_states` | `sensor_msgs/JointState` | 臂 → Agent | 关节角度反馈 |
| `/arm/joint_command` | `std_msgs/Float64MultiArray` | Agent → 臂 | 关节角度指令 |
| `/arm/end_effector_id` | `std_msgs/UInt8` | 末端识别 → Agent | 末端模块类型 ID |
| `/arm/gripper_state` | `std_msgs/Bool` | 臂 → Agent | 夹爪开合状态 |

**L3 视觉感知**

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/vision/target_pose` | `geometry_msgs/PoseStamped` | 感知 → Agent | 目标物位姿 |
| `/vision/detections` | `自定义 Detection2DArray` | 感知 → Agent/调试 | 检测结果数组 |
| `/vision/debug_image` | `sensor_msgs/Image` | 感知 → 调试面板 | 标注后调试图像 |

**L2 Agent 调度层**

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/agent/task_state` | `std_msgs/String` | Agent → MR/调试 | 当前状态机节点名 |
| `/agent/task_feedback` | `自定义 TaskFeedback` | Agent → MR | 任务进度百分比 |

**L1 BCI 意图接入层**

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/bci/attention_level` | `std_msgs/Float32` | BCI → Agent | 专注度 0~100 |
| `/bci/task_trigger` | `std_msgs/String` | BCI → Agent | START / PAUSE / ABORT |

**L4 MR 空间反馈层**

| 话题 | 消息类型 | 方向 | 说明 |
|------|----------|------|------|
| `/mr/robot_state` | `自定义 RobotStateMsg` | Agent → MR | 位姿+状态+路径打包 |
| `/mr/path_preview` | `nav_msgs/Path` | Nav2 → MR | 规划路径预览 |
| `/mr/operator_gesture` | `std_msgs/String` | MR → Agent | 操作者手势确认（扩展） |

### 4.2 话题命名规则

```
✅  /arm/joint_states          — 命名空间 + 数据名
✅  /vision/target_pose        — 功能域 + 语义
✅  /bci/attention_level       — 层级清晰

❌  /ArmJointStates            — 禁止大写
❌  /arm-joint-states          — 禁止连字符
❌  /data                      — 语义不明
❌  /my_topic_1                — 无意义编号
```

---

## 5 服务（Service）

格式：`/<命名空间>/<动作_对象>`

```
/arm/set_gripper             # 设置夹爪开合
/arm/go_home                 # 机械臂归零
/chassis/set_speed_factor    # 设置速度修正系数
/agent/abort_task            # 中止当前任务
/vision/trigger_detection    # 手动触发一次检测
/bci/set_threshold           # 设置专注度阈值
/system/get_diagnostics      # 获取系统诊断信息
```

---

## 6 动作（Action）

格式：`/<命名空间>/<持续性任务名>`

```
/navigation/navigate_to_pose     # Nav2 标准导航
/arm/execute_grasp               # 执行抓取序列
/arm/execute_place               # 执行放置序列
/agent/execute_transport_task    # 执行完整搬运任务
```

动作定义文件命名用 `PascalCase`：

```
NavigateToPose.action
ExecuteGrasp.action
ExecutePlace.action
TransportTask.action
```

---

## 7 自定义消息/服务/动作类型

格式：**PascalCase**，文件扩展名 `.msg` / `.srv` / `.action`

```
# msg/
Detection2D.msg              # 单个检测结果
Detection2DArray.msg         # 检测结果数组
RobotStateMsg.msg            # MR 用打包状态
TaskFeedback.msg             # 任务进度反馈
EndEffectorId.msg            # 末端模块 ID

# srv/
SetGripper.srv               # 设置夹爪
SetThreshold.srv             # 设置阈值
GetDiagnostics.srv           # 获取诊断

# action/
ExecuteGrasp.action          # 抓取动作
TransportTask.action         # 搬运任务
```

消息字段用 `snake_case`：

```
# Detection2D.msg
string class_name
float32 confidence
float32 center_x
float32 center_y
float32 width
float32 height
```

---

## 8 TF 坐标系（Frame ID）

格式：`snake_case`，无 `/` 前缀

```
                    map
                     │
                  odom
                     │
               base_link
              ┌────┬────┐
        left_wheel  right_wheel
              │
         base_imu_link
              │
         laser_link
              │
         camera_link
              │
        camera_optical_link
              │
         arm_base_link
           │
      arm_link_1 → arm_link_2 → ... → arm_end_effector_link
```

| Frame | 说明 |
|-------|------|
| `map` | 全局地图坐标系（SLAM 输出） |
| `odom` | 里程计坐标系 |
| `base_link` | 底盘几何中心 |
| `base_footprint` | 底盘投影到地面（Nav2 用） |
| `laser_link` | RPLIDAR S1 安装位 |
| `camera_link` | Orbbec Astra Pro 深度相机安装位 |
| `camera_optical_link` | 摄像头光学坐标系（Z 朝前） |
| `arm_base_link` | 机械臂基座 |
| `arm_link_N` | 第 N 个关节 |
| `arm_end_effector_link` | 末端执行器法兰 |
| `base_imu_link` | IMU 安装位 |

---

## 9 参数（Parameter）

格式：`snake_case`，用 `.` 分层级（YAML 中自然对应嵌套）

```yaml
# chassis_params.yaml
chassis:
  wheel_track: 0.160          # 轮距 (m)
  wheel_diameter: 0.065       # 轮径 (m)
  gear_ratio: 30
  encoder_ppr: 11             # 磁环线数
  i2c_bus: 5
  i2c_address: 0x26
  motor_left_channel: 3       # M3
  motor_right_channel: 2      # M2
  speed_correction_left: 1.5
  speed_correction_right: 1.5
  max_linear_speed: 0.5       # m/s
  max_angular_speed: 2.0      # rad/s

# bci_params.yaml
bci:
  device_type: "neurosky"     # neurosky / emotiv
  attention_threshold: 70     # 触发阈值
  trigger_hold_duration: 2.0  # 持续秒数
  fallback_keyboard: true     # 键盘备用

# mr_params.yaml
mr:
  websocket_port: 9090
  refresh_rate_hz: 10
  path_preview_color: [0.0, 1.0, 0.4, 0.8]  # RGBA
```

---

## 10 Launch 文件

格式：`<功能>_launch.py`（ROS2 推荐 Python launch）

```
bringup_launch.py            # 全系统启动
chassis_launch.py            # 仅底盘 + 传感器
navigation_launch.py         # SLAM + Nav2
perception_launch.py         # 摄像头 + 检测器
arm_launch.py                # 机械臂节点
agent_launch.py              # 任务调度 + 行为树
bci_launch.py                # BCI 接口
mr_bridge_launch.py          # MR WebSocket 桥
simulation_launch.py         # Gazebo 仿真
```

---

## 11 Python 源文件与类

文件名 `snake_case.py`，类名 `PascalCase`：

```
chassis_driver.py        → class ChassisDriver
imu_driver.py            → class ImuDriver
lidar_driver.py          → class LidarDriver
camera_driver.py         → class CameraDriver
arm_driver.py            → class ArmDriver
detector.py              → class ObjectDetector
calibrator.py            → class HandEyeCalibrator
task_scheduler.py        → class TaskScheduler
behavior_tree_node.py    → class BehaviorTreeExecutor
bci_interface.py         → class BciInterface
mr_interface.py          → class MrBridgeInterface
config_loader.py         → class ConfigLoader
```

函数和变量用 `snake_case`：

```python
def get_wheel_speed(self) -> Tuple[float, float]: ...
def set_target_velocity(self, linear: float, angular: float): ...

current_state = TaskState.IDLE
attention_level = 0.0
```

常量用 `UPPER_SNAKE_CASE`：

```python
MAX_LINEAR_SPEED = 0.5
DEFAULT_ATTENTION_THRESHOLD = 70
I2C_MOTOR_ADDRESS = 0x26
```

---

## 12 配置文件与目录

```
config/
├── chassis_params.yaml
├── imu_params.yaml
├── lidar_params.yaml
├── camera_params.yaml
├── arm_params.yaml
├── nav2_params.yaml
├── slam_params.yaml
├── bci_params.yaml
├── mr_params.yaml
└── detection_params.yaml
```

URDF / Xacro 文件：

```
urdf/
├── agent_agv.urdf.xacro       # 顶层描述
├── chassis.urdf.xacro          # 底盘
├── sensors.urdf.xacro          # 传感器
└── arm.urdf.xacro              # 机械臂（待型号确认）
```

---

## 13 Git 分支与 Commit

分支格式：`<类型>/<简述>`

```
feat/slam-nav2-integration
feat/arm-driver
feat/bci-interface
fix/odom-drift-correction
refactor/chassis-ros2-node
docs/naming-convention
```

Commit 格式：`<type>(<scope>): <description>`

```
feat(chassis): wrap ChassisDriver as ROS2 node
fix(imu): correct quaternion frame orientation
docs(readme): update hardware parameter table
refactor(vision): split detector into color and shape modules
test(arm): add joint limit boundary test
```

---

## 14 快速检查清单

- [ ] 所有名称仅含 `a-z 0-9 _`，以字母开头
- [ ] 话题按 `/<层级命名空间>/<语义名>` 组织
- [ ] TF frame 无 `/` 前缀
- [ ] 自定义消息类型用 PascalCase，字段用 snake_case
- [ ] Python 文件 snake_case，类 PascalCase，常量 UPPER_SNAKE_CASE
- [ ] 参数 YAML 嵌套结构清晰，键名 snake_case
- [ ] Launch 文件以 `_launch.py` 结尾
- [ ] Git commit 遵循 conventional commits