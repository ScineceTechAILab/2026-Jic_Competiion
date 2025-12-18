import time
import struct
from smbus2 import SMBus

# ==========================================
# 4路电机驱动板 I2C 控制协议
# ==========================================

# I2C 总线号
BUS_NUM = 5
# 设备地址 (根据文档为0x26)
DEVICE_ADDR = 0x26

# 寄存器地址 (根据文档)
REG_MOTOR_TYPE = 0x01      # 电机类型配置 (只写)
REG_DEADZONE = 0x02        # 死区配置 (只写)
REG_MAGNETIC_LINES = 0x03  # 磁环线数 (只写)
REG_REDUCTION_RATIO = 0x04 # 减速比 (只写)
REG_WHEEL_DIAMETER = 0x05  # 轮子直径 (只写)
REG_SPEED_CONTROL = 0x06   # 速度控制 (只写)
REG_PWM_CONTROL = 0x07     # PWM控制 (只写)
REG_BATTERY_VOLTAGE = 0x08 # 读取电池电量 (只读)
REG_M1_ENCODER = 0x10      # 读取M1编码器数据 (只读)
REG_M2_ENCODER = 0x12      # 读取M2编码器数据 (只读) - 根据文档0x12寄存器

def main():
    print(f"正在打开 I2C Bus {BUS_NUM}...")
    try:
        bus = SMBus(BUS_NUM)
    except Exception as e:
        print(f"无法打开总线: {e}")
        return

    print("开始测试：向 0x26 发送4路电机驱动板协议指令...")
    print("专门测试M2电机（左下电机/左后轮）")
    
    # 先读取电池电量测试I2C读取功能
    def read_battery_voltage():
        try:
            # 读取电池电量寄存器 (0x08)
            # 根据文档：data = (buf[0] <<8|buf[1]/10
            data = bus.read_i2c_block_data(DEVICE_ADDR, REG_BATTERY_VOLTAGE, 2)
            if len(data) >= 2:
                voltage = (data[0] << 8 | data[1]) / 10.0
                print(f"电池电压: {voltage:.2f}V")
                return voltage
            else:
                print("读取电池电量失败：数据长度不足")
                return None
        
        except OSError as e:
            print(f"读取电池电量失败: {e}")
            return None

    # 读取M2编码器数据（根据文档0x12寄存器）
    def read_m2_encoder():
        try:
            # 读取M2编码器寄存器 (0x12)
            # 根据文档：data = buf[0] <<8|buf
            data = bus.read_i2c_block_data(DEVICE_ADDR, REG_M2_ENCODER, 2)
            if len(data) >= 2:
                encoder_value = (data[0] << 8 | data[1])
                # 处理有符号数（如果最高位是1，则为负数）
                if encoder_value & 0x8000:
                    encoder_value = encoder_value - 0x10000
                print(f"M2编码器值: {encoder_value}")
                return encoder_value
            else:
                print("读取M2编码器失败：数据长度不足")
                return None
        except OSError as e:
            print(f"读取M2编码器失败: {e}")
            return None

    # 只控制M2电机的PWM（其他电机保持为0）
    def set_m2_pwm(pwm_value):
        # 限制PWM范围: -3600 到 3600
        if pwm_value > 3600: 
            pwm_value = 3600
        elif pwm_value < -3600: 
            pwm_value = -3600
        
        # 转换为大端模式的int16_t数组（只设置M2，其他为0）
        pwm_int = int(pwm_value)
        data_bytes = bytearray()
        
        # M1: 0
        data_bytes.extend(struct.pack('>h', 0))
        # M2: 指定值
        data_bytes.extend(struct.pack('>h', pwm_int))
        # M3: 0
        data_bytes.extend(struct.pack('>h', 0))
        # M4: 0
        data_bytes.extend(struct.pack('>h', 0))
        
        try:
            # 写入PWM控制寄存器 (0x07)
            bus.write_i2c_block_data(DEVICE_ADDR, REG_PWM_CONTROL, list(data_bytes))
            print(f"设置M2 PWM值: {pwm_value}")
            return True
        except OSError as e:
            print(f"I2C 写入失败: {e}")
            return False

    # 只控制M2电机的速度（其他电机保持为0）
    def set_m2_speed(speed_value):
        # 限制速度范围: -1000 到 1000
        if speed_value > 1000: 
            speed_value = 1000
        elif speed_value < -1000: 
            speed_value = -1000
        
        # 转换为大端模式的int16_t数组（只设置M2，其他为0）
        speed_int = int(speed_value)
        data_bytes = bytearray()
        
        # M1: 0
        data_bytes.extend(struct.pack('>h', 0))
        # M2: 指定值
        data_bytes.extend(struct.pack('>h', speed_int))
        # M3: 0
        data_bytes.extend(struct.pack('>h', 0))
        # M4: 0
        data_bytes.extend(struct.pack('>h', 0))
        
        try:
            # 写入速度控制寄存器 (0x06)
            bus.write_i2c_block_data(DEVICE_ADDR, REG_SPEED_CONTROL, list(data_bytes))
            print(f"设置M2速度值: {speed_value}")
            return True
        except OSError as e:
            print(f"I2C 写入失败: {e}")
            return False

    # 配置M2电机参数（测试用）
    def config_m2_motor():
        try:
            print("\n=== 配置M2电机参数 ===")
            
            # 1. 配置电机类型（假设使用520电机）
            # 根据文档：1=520电机，2=310电机，3=TT电机（带编码器），4=TT电机（不带编码器）
            print("1. 配置电机类型为520电机...")
            bus.write_byte_data(DEVICE_ADDR, REG_MOTOR_TYPE, 1)
            print("  电机类型配置发送完成")
            time.sleep(0.5)
            
            # 2. 配置死区（默认值）
            print("2. 配置死区值为1600...")
            deadzone_data = struct.pack('>h', 1600)
            bus.write_i2c_block_data(DEVICE_ADDR, REG_DEADZONE, list(deadzone_data))
            print("  死区配置发送完成")
            time.sleep(0.5)
            
            # 3. 配置相位线（根据文档默认11）
            print("3. 配置相位线为11...")
            bus.write_byte_data(DEVICE_ADDR, REG_MAGNETIC_LINES, 11)
            print("  相位线配置发送完成")
            time.sleep(0.5)
            
            # 4. 配置减速比（根据文档默认30）
            print("4. 配置减速比为30...")
            bus.write_byte_data(DEVICE_ADDR, REG_REDUCTION_RATIO, 30)
            print("  减速比配置发送完成")
            time.sleep(0.5)
            
            print("电机参数配置完成！")
            return True
            
        except OSError as e:
            print(f"电机配置失败: {e}")
            return False

    try:
        print("=== M2电机测试开始 ===")
        
        # 1. 先读取电池电量，确认I2C通信正常
        print("\n1. 读取电池电量...")
        voltage = read_battery_voltage()
        if voltage is None:
            print("警告：无法读取电池电量，但继续测试...")
        
        # 2. 读取M2编码器数据（如果电机有编码器）
        print("\n2. 读取M2编码器初始数据...")
        initial_encoder = read_m2_encoder()
        
        # 3. 可选：配置M2电机参数
        print("\n3. 配置M2电机参数（可选）...")
        config_m2_motor()
        
        # 4. PWM控制测试 - 正向转动
        print("\n4. PWM控制测试 - M2正向转动")
        print("   预计M2电机（左下/左后轮）会正向转动")
        if set_m2_pwm(1000):
            print("   PWM控制发送成功，电机应该开始转动")
            time.sleep(3)
        else:
            print("   PWM控制发送失败")
        
        # 5. 读取编码器数据变化
        print("\n5. 读取M2编码器数据（转动中）...")
        read_m2_encoder()
        
        # 6. 停止电机
        print("\n6. 停止M2电机...")
        set_m2_pwm(0)
        print("   电机应停止转动")
        time.sleep(2)
        
        # 7. PWM控制测试 - 反向转动
        print("\n7. PWM控制测试 - M2反向转动")
        print("   预计M2电机会反向转动")
        if set_m2_pwm(-800):
            print("   PWM控制发送成功，电机应该开始反向转动")
            time.sleep(3)
        else:
            print("   PWM控制发送失败")
        
        # 8. 停止电机
        print("\n8. 停止M2电机...")
        set_m2_pwm(0)
        time.sleep(2)
        
        # 9. 读取最终编码器数据
        print("\n9. 读取M2编码器最终数据...")
        final_encoder = read_m2_encoder()
        
        # 10. 尝试速度控制（如果电机带编码器）
        print("\n10. 速度控制测试（只对带编码器电机有效）...")
        if set_m2_speed(300):
            print("   速度控制发送成功，如果电机带编码器，会以300速度运行")
            time.sleep(2)
            
            # 读取编码器看速度控制效果
            print("   读取M2编码器数据（速度控制中）...")
            read_m2_encoder()
            
            # 停止
            set_m2_speed(0)
            print("   速度控制停止")
        else:
            print("   速度控制发送失败")
        
        # 11. 最终停止所有电机
        print("\n11. 确保所有电机停止...")
        # 发送全零的PWM控制
        bus.write_i2c_block_data(DEVICE_ADDR, REG_PWM_CONTROL, [0, 0, 0, 0, 0, 0, 0, 0])
        print("   所有电机已停止")
        
        print("\n=== M2电机测试结束 ===")
        print("测试总结：")
        print(f"- 电池电压: {voltage if voltage else '读取失败'}")
        print(f"- M2编码器初始值: {initial_encoder}")
        print(f"- M2编码器最终值: {final_encoder}")
        
    except KeyboardInterrupt:
        # 紧急停止
        print("\n紧急停止所有电机...")
        # 发送全零的PWM控制
        try:
            bus.write_i2c_block_data(DEVICE_ADDR, REG_PWM_CONTROL, [0, 0, 0, 0, 0, 0, 0, 0])
        except:
            pass
        print("已停止所有电机")
    finally:
        # 关闭I2C总线
        try:
            bus.close()
        except:
            pass

if __name__ == "__main__":
    main()
