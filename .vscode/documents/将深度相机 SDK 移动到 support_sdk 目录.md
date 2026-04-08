# 迁移深度相机 SDK 至 Support 层

这是一个很好的建议。将第三方 SDK 与我们自己编写的业务代码分离，符合“高内聚、低耦合”的设计原则。

*   **`src/support/sdk/`**: 用于存放第三方的 SDK 或库文件（如 `OrbbecSDK_ROS2`），作为项目的底层依赖。
*   **`src/execution/camera_control/`**: 保持保留，用于存放**我们自己编写的**相机控制逻辑、特定任务的 Launch 文件或参数配置，它将调用 `support` 层中的 SDK。

## 执行步骤

1.  **创建目录**:
    *   创建 `src/support/sdk/` 目录。

2.  **移动文件**:
    *   将 `src/execution/camera_control/OrbbecSDK_ROS2` 移动到 `src/support/sdk/OrbbecSDK_ROS2`。

3.  **更新文档**:
    *   修改 `docs/dev_instruction/project_structure.md`，更新 SDK 的位置说明。

4.  **清理环境**:
    *   如果 `src/execution/camera_control/` 变为空，保留该目录（因为后续会用于存放我们自己的代码）。

5.  **验证构建**:
    *   运行 `colcon build` 确保移动后构建系统仍能正确识别该 ROS 2 包（`colcon` 会递归扫描 `src` 目录，所以理论上不会有影响）。