I will implement the LiDAR driver node integration by leveraging the official `sllidar_ros2` driver, as it provides the most robust support for RPLIDAR S1.

### 1. Environment Setup (init.bash)
- Add a command to `init.bash` to clone the `sllidar_ros2` repository into `src/support/sdk/`. This ensures the driver is available in the workspace.
- **Command**: `git clone https://github.com/Slamtec/sllidar_ros2.git src/support/sdk/sllidar_ros2`

### 2. Create `lidar_control` Package
- Create a new ROS 2 Python package at `src/execution/lidar_control`.
- This package will serve as the execution layer interface for the LiDAR, wrapping the low-level driver.

### 3. Implement Launch Logic
- Create `src/execution/lidar_control/launch/lidar.launch.py`.
- This launch file will:
    - Include/Launch the `sllidar_node` from the `sllidar_ros2` package.
    - Configure it for **RPLIDAR S1** with the specifications from `hardware_config.md`:
        - Serial Port: `/dev/ttyUSB0` (default, configurable)
        - Baudrate: `115200` (Note: S1 standard is often 256000, but I will set default to 115200 as per project docs, with a comment).
        - Frame ID: `laser_link` (or `lidar_link` to match TF tree).

### 4. Documentation
- Update `docs/dev_diary/2025.1.24.md` to mark the LiDAR node task as completed.
