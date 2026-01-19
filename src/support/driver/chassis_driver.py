
"""
ChassisDriver – I²C 4-channel motor driver board control module (differential drive)

This module provides a high-level interface for a 4-wheel motor driver board
communicating over I²C.  It supports:

* Motor parameter configuration (type, reduction ratio, encoder lines, dead-zone)
* Closed-loop speed control in RPM
* Raw PWM control
* Differential-drive kinematics (linear + angular velocity)
* Battery voltage monitoring
* Flexible motor-ID mapping and direction inversion

Usage
-----
>>> chassis = ChassisDriver(bus_num=5, addr=0x26)
>>> chassis.configure_motors(motor_type=1, reduction_ratio=30, magnetic_lines=11)
>>> chassis.set_movement(linear_x=0.5, angular_z=0.0)   # 0.5 m/s straight
>>> chassis.stop()

Author
------
Your Name <your.email@example.com>

License
-------
MIT
"""


import time
import struct
import math
from smbus2 import SMBus

class ChassisDriver:
    
    """
    4路电机驱动板控制类 (差速模型)
    """
    
    # 寄存器定义
    REG_MOTOR_TYPE = 0x01
    REG_DEADZONE = 0x02
    REG_MAGNETIC_LINES = 0x03
    REG_REDUCTION_RATIO = 0x04
    REG_WHEEL_DIAMETER = 0x05
    REG_SPEED_CONTROL = 0x06
    REG_PWM_CONTROL = 0x07
    REG_BATTERY_VOLTAGE = 0x08
    
    # 编码器寄存器基地址 (M1=0x10, M2=0x12, M3=0x14, M4=0x16)
    REG_ENCODER_BASE = 0x10
    
    def __init__(self, bus_num=5, addr=0x26, wheel_diameter_mm=65, wheel_base_mm=160):
        """
        初始化驱动板
        :param bus_num: I2C总线号
        :param addr: 设备地址
        :param wheel_diameter_mm: 轮子直径 (mm)
        :param wheel_base_mm: 轮距 (mm, 两个驱动轮之间的距离)
        """
        self.bus_num = bus_num
        self.addr = addr
        self.wheel_diameter_mm = wheel_diameter_mm
        self.wheel_base_mm = wheel_base_mm
        self.bus = None
        
        # 默认电机映射 (用户需要根据实际接线修改)
        # 假设: M2=左后, M1=右后 (仅为示例，需通过测试确认)
        self.left_motor_id = 2 
        self.right_motor_id = 1
        
        # 电机方向修正 (1 或 -1)
        self.left_motor_dir = 1
        self.right_motor_dir = 1
        
        # 速度修正系数 (默认为1.0)
        self.left_scale = 1.0
        self.right_scale = 1.0
        
        self._connect()

    def set_speed_correction(self, left_scale=1.0, right_scale=1.0):
        """
        设置速度修正系数
        :param left_scale: 左轮速度缩放系数
        :param right_scale: 右轮速度缩放系数
        """
        self.left_scale = left_scale
        self.right_scale = right_scale

    def _connect(self):
        try:
            self.bus = SMBus(self.bus_num)
        except Exception as e:
            print(f"I2C Bus {self.bus_num} connection failed: {e}")

    def close(self):
        if self.bus:
            self.bus.close()

    def set_motor_mapping(self, left_id, right_id, left_dir=1, right_dir=1):
        """
        设置电机映射和方向
        
        - param left_id: 左轮电机ID (1-4)
        - param right_id: 右轮电机ID (1-4)
        - param left_dir: 左轮方向修正 (1 或 -1)
        - param right_dir: 右轮方向修正 (1 或 -1)

        """
        self.left_motor_id = left_id
        self.right_motor_id = right_id
        self.left_motor_dir = left_dir
        self.right_motor_dir = right_dir

    def configure_motors(self, motor_type=1, reduction_ratio=30, magnetic_lines=11, deadzone=1600):
        """
        配置电机参数 (假设左右轮使用相同电机)
        :param motor_type: 1=520, 2=310, 3=TT(Enc), 4=TT(NoEnc)
        :param reduction_ratio: 减速比
        :param magnetic_lines: 磁环线数
        :param deadzone: 死区
        """
        if not self.bus: return False
        
        try:
            # 1. 类型
            self.bus.write_byte_data(self.addr, self.REG_MOTOR_TYPE, motor_type)
            time.sleep(0.1)
            
            # 2. 死区
            self.bus.write_i2c_block_data(self.addr, self.REG_DEADZONE, list(struct.pack('>h', deadzone)))
            time.sleep(0.1)
            
            # 3. 线数
            self.bus.write_byte_data(self.addr, self.REG_MAGNETIC_LINES, magnetic_lines)
            time.sleep(0.1)
            
            # 4. 减速比
            self.bus.write_byte_data(self.addr, self.REG_REDUCTION_RATIO, reduction_ratio)
            time.sleep(0.1)
            
            return True
        except OSError as e:
            print(f"Config failed: {e}")
            return False

    def get_battery_voltage(self):
        if not self.bus: return 0.0
        try:
            data = self.bus.read_i2c_block_data(self.addr, self.REG_BATTERY_VOLTAGE, 2)
            if len(data) >= 2:
                return (data[0] << 8 | data[1]) / 10.0
        except OSError:
            pass
        return 0.0

    def _set_motors_speed(self, speeds):
        """
        底层发送4路电机速度
        :param speeds: 长度为4的列表/元组，对应 [M1, M2, M3, M4] 的速度值
        """
        if not self.bus: return
        
        data_bytes = bytearray()
        for s in speeds:
            # 限制范围 -1000 ~ 1000
            s = max(-1000, min(1000, int(s)))
            data_bytes.extend(struct.pack('>h', s))
            
        try:
            self.bus.write_i2c_block_data(self.addr, self.REG_SPEED_CONTROL, list(data_bytes))
        except OSError as e:
            print(f"Write speed failed: {e}")

    def set_movement(self, linear_x, angular_z):
        """
        差速运动控制
        :param linear_x: 线速度 (m/s), 前进为正
        :param angular_z: 角速度 (rad/s), 左转(逆时针)为正
        """
        # 差速运动学公式
        # v_L = v - (omega * L / 2)
        # v_R = v + (omega * L / 2)
        # 注意单位转换: 我们需要将 m/s 转换为驱动板的速度单位
        # 假设驱动板的速度单位大致对应 RPM 或 某种编码器tick率
        # 这里我们需要一个转换系数。通常驱动板接收的是 RPM 或者 PWM。
        # 如果是 RPM: v (m/s) = (RPM / 60) * pi * D
        # -> RPM = v * 60 / (pi * D)
        
        # 轮距 (m)
        L = self.wheel_base_mm / 1000.0
        
        v_l_mps = linear_x - (angular_z * L / 2.0)
        v_r_mps = linear_x + (angular_z * L / 2.0)
        
        # 转换为 RPM (假设驱动板寄存器接受 RPM，需根据实际情况校准)
        # 轮子直径 (m)
        D = self.wheel_diameter_mm / 1000.0
        
        rpm_l = (v_l_mps * 60) / (math.pi * D)
        rpm_r = (v_r_mps * 60) / (math.pi * D)
        
        # 应用方向修正和速度修正
        target_speed_l = rpm_l * self.left_motor_dir * self.left_scale
        target_speed_r = rpm_r * self.right_motor_dir * self.right_scale
        
        # 构建4路电机速度数组
        speeds = [0, 0, 0, 0]
        
        # 映射到对应的ID (ID从1开始，数组索引从0开始)
        if 1 <= self.left_motor_id <= 4:
            speeds[self.left_motor_id - 1] = target_speed_l
            
        if 1 <= self.right_motor_id <= 4:
            speeds[self.right_motor_id - 1] = target_speed_r
            
        self._set_motors_speed(speeds)

    def _set_motors_pwm(self, pwms):
        """
        底层发送4路电机PWM
        :param pwms: 长度为4的列表/元组，对应 [M1, M2, M3, M4] 的PWM值
        """
        if not self.bus: return
        
        data_bytes = bytearray()
        for p in pwms:
            # 限制范围 -3600 ~ 3600
            p = max(-3600, min(3600, int(p)))
            data_bytes.extend(struct.pack('>h', p))
            
        try:
            self.bus.write_i2c_block_data(self.addr, self.REG_PWM_CONTROL, list(data_bytes))
        except OSError as e:
            print(f"Write PWM failed: {e}")

    def stop(self):
        self._set_motors_speed([0, 0, 0, 0])
        self._set_motors_pwm([0, 0, 0, 0])


# TODO: 完成底盘驱动运动逆运动学矩阵方程
# TODO：人工复核代码并且精修代码