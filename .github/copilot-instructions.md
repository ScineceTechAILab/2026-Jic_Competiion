# GitHub Copilot Instructions for 嘉立创地瓜机器人 (JIC Competition Robot)

## Project Overview

This is a ROS2-based robotics competition project ("Sweet Potato Robot") targeting the **D-Robotics RDK X5 platform** (formerly Horizon Robotics). The robot integrates vision AI (BPU hardware accelerator), motor control, LIDAR, and cameras for autonomous navigation and manipulation tasks.

**Platform:** D-Robotics RDK X5 with Sunrise X5 SoC  
**Hardware Stack:** I2C motor controllers (0x26 @ Bus 5), UART LIDAR (思岚S1), USB/MIPI cameras, 40-pin GPIO  
**Software Stack:** ROS2 (ament_cmake), Python 3.x, PyTorch inference via `hobot_dnn`, OpenCV

## Architecture

### Layered Design (`src/` directory structure)

```
application/     # High-level task planning and BCI interaction
  ├── bci_mr_interaction/  # Brain-computer interface & mixed reality
  └── task_planner/        # Mission orchestration

decision/        # Path planning and behavior trees
  ├── behavior_tree/
  └── path_planning/

execution/       # Hardware control modules
  ├── chassis_control/     # 4-motor I2C driver (0x26)
  ├── camera_control/
  ├── arm_control/
  ├── end_effector/
  └── lidar_control/

functional/      # Perception and SLAM
  ├── visual_perception/   # BPU-accelerated inference
  ├── slam_module/
  └── obstacle_avoidance/

support/         # Utilities and simulation
  ├── ros2_utils/
  └── simulation/
```

### Critical Hardware Interfaces

1. **Motor Control:** 4-channel motor driver board via I2C Bus 5 (0x26)
   - Registers: `0x06` (speed control, -1000~1000), `0x07` (PWM control, -3600~3600)
   - Encoders: M1(0x10), M2(0x12), M3(0x14), M4(0x16) - 16-bit signed int, big-endian
   - Battery: `0x08` returns voltage×10 (read 2 bytes, divide by 10)
   - **Must write all 4 motors** in single transaction (8 bytes total)
   - See: `tests/chassis/test_motor_m2.py` for working examples

2. **LIDAR:** 思岚S1 via UART @ 256000 baud
   - Test ports: `/dev/ttyUSB0`, `/dev/ttyS2` (UART3 on RDK X5)
   - Protocol: Custom binary format (see `tests/lidar/test_lidar.py` for parser)

3. **Vision:**
   - USB cameras: `/dev/video*` (OpenCV via `cv2.VideoCapture`)
   - MIPI cameras: Native SDK (`hobot_dnn`, `pydev_demo/03_mipi_camera_sample/`)
   - BPU inference: `from hobot_dnn import pyeasy_dnn as dnn` for models in `model/` or `pydev_demo/models`

4. **GPIO:** `Hobot.GPIO` library (BOARD mode, e.g., pin 37 for outputs)

## Development Workflow

### Environment Setup
```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install -r requirements-dev.txt  # If exists, for testing
```

### Testing
- **Unit tests:** `pytest` with hardware mocking (see `tests/README.md`)
- **Hardware tests:** Run scripts in `tests/{chassis,lidar,camera}/` on device
- Run all tests: `pytest --cov=src`

### Code Conventions

1. **Module structure:**
   - `*/include/<pkg>/`: Reusable library code with `__init__.py` (e.g., `chassis_control/i2c_driver.py`)
   - `*/scripts/`: Entry points only - parse args, call `include` modules
   - `*/launch/`: ROS2 launch files for that subsystem

2. **Hardware mocking:** Use `pytest-mock` to monkeypatch `smbus2`, `serial`, `Hobot.GPIO` in tests

3. **Import style:** `from chassis_control.i2c_driver import ChassisDriver` (not relative imports)

4. **Linting:** Use `flake8` and `mypy` before commits

### ROS2 Patterns

- Launch files use `generate_launch_description()` with `IncludeLaunchDescription`
- Config files: `config/ros2_settings.yaml` (DDS), `config/system_params.yaml` (sensors)
- Main launch: `launch/full_system.launch.py` orchestrates all subsystems

## Platform-Specific Notes

### D-Robotics RDK X5 SDK

- **Multimedia samples:** `multimedia_samples/` contains C examples for VIO/ISP/codec
- **Python inference:** `pydev_demo/` shows BPU usage (models must be `.bin` format from Horizon toolchain)
- **Camera pipeline:** Use `sunrise_camera` SDK for MIPI sensors, or standard v4l2 for USB
- **I2C bus enumeration:** Bus 5 is for motor controller - verify with `i2cdetect -y 5`

### Common Pitfalls

1. **I2C writes fail silently:** Always check bus number (5, not 0/1) and verify device with `i2cdetect`
2. **Serial port permissions:** Add user to `dialout` group or run with sudo for `/dev/ttyUSB*`
3. **Camera conflicts:** X5 reserves some `/dev/video` nodes for ISP - use `v4l2-ctl --list-devices`
4. **BPU models:** Standard PyTorch/ONNX won't work - must convert with Horizon toolchain first

## Key Files Reference

- `doc/devolop/项目结构.md` - Architecture guidelines (Chinese)
- `CONTRIBUTING.md` - PR workflow and CI setup
- `tests/chassis/test_motor_m2.py` - I2C motor control example with encoder feedback
- `tests/lidar/test_lidar.py` - UART LIDAR protocol implementation
- `tests/camera/test_usb_camera.py` - Camera detection and OpenCV integration
- `40pin_samples/` - GPIO examples for RDK X5 40-pin header

## External Dependencies

- `smbus2`: I2C communication (no `python-smbus` - deprecated)
- `pyserial`: UART devices
- `Hobot.GPIO`: Platform-specific GPIO (RDK X5 equivalent of RPi.GPIO)
- `hobot_dnn`: BPU inference engine (proprietary D-Robotics SDK)
- `adafruit-circuitpython-*`: Servo/PWM libraries for add-on boards

## When Adding Features

1. **New hardware:** Create test in `tests/<subsystem>/` first with direct SDK calls
2. **Refactor to library:** Move logic to `src/execution/<module>/include/<pkg>/driver.py`
3. **Add unit tests:** Mock hardware dependencies with `monkeypatch` in `tests/unit/`
4. **Create ROS2 node:** Wrapper in `scripts/` that imports driver and publishes topics
5. **Update launch:** Add to relevant `launch/*.py` and `launch/full_system.launch.py`

## Notes on Chinese Comments

Most inline comments are in Chinese (project requirement). When reading:
- 电机 = motor, 速度 = speed, 编码器 = encoder
- 驱动 = driver, 总线 = bus, 寄存器 = register
- 测试 = test, 读取 = read, 写入 = write
