import time
import struct
from smbus2 import SMBus

# ==========================================
# 4路电机驱动板 I2C 控制协议 - M3电机测试
# ==========================================

# I2C 配置
BUS_NUM = 5          # 根据您的树莓派或主控的I2C总线号修改，常见的是1或5
DEVICE_ADDR = 0x26   # 驱动板I2C设备地址

# 寄存器地址 (根据文档)
REG_MOTOR_TYPE = 0x01      # 电机类型配置 (只写)
REG_DEADZONE = 0x02        # 死区配置 (只写)
REG_MAGNETIC_LINES = 0x03  # 磁环线数 (只写)
REG_REDUCTION_RATIO = 0x04 # 减速比 (只写)
REG_WHEEL_DIAMETER = 0x05  # 轮子直径 (只写)
REG_SPEED_CONTROL = 0x06   # 速度控制 (只写，仅对带编码器电机有效)
REG_PWM_CONTROL = 0x07     # PWM控制 (只写，对所有电机有效)
REG_BATTERY_VOLTAGE = 0x08 # 读取电池电量 (只读)
REG_M3_ENCODER = 0x12      # 读取M3编码器实时脉冲数据 (只读)

def main():
    print(f"正在打开 I2C Bus {BUS_NUM}...")
    try:
        bus = SMBus(BUS_NUM)
    except Exception as e:
        print(f"无法打开总线: {e}")
        return

    print("开始测试：向地址 0x26 发送4路电机驱动板协议指令...")
    print("测试对象：M3电机（右上电机/右前轮）")
    
    # 1. 读取电池电量，测试基础通信
    def read_battery_voltage():
        try:
            # 读取电池电量寄存器 (0x08)
            # 根据文档：data = (buf[0] << 8 | buf[1]) / 10
            data = bus.read_i2c_block_data(DEVICE_ADDR, REG_BATTERY_VOLTAGE, 2)
            if len(data) >= 2:
                voltage = ((data[0] << 8) | data[1]) / 10.0
                print(f"[状态] 电池电压: {voltage:.2f} V")
                return voltage
            else:
                print("[错误] 读取电池电量失败：数据长度不足")
                return None
        except OSError as e:
            print(f"[错误] 读取电池电量失败: {e}")
            return None

    # 2. 读取M3编码器数据
    def read_m3_encoder():
        try:
            # 读取M3编码器寄存器 (0x12)
            # 根据文档：data = buf[0] <<8|buf
            data = bus.read_i2c_block_data(DEVICE_ADDR, REG_M3_ENCODER, 2)
            if len(data) >= 2:
                encoder_value = (data[0] << 8 | data[1] )
                # 处理有符号数（如果最高位是1，则为负数）
                if encoder_value & 0x8000:
                    encoder_value = encoder_value - 0x10000
                print(f"[状态] M3编码器实时脉冲值: {encoder_value}")
                return encoder_value
            else:
                print("[错误] 读取M3编码器失败：数据长度不足")
                return None
        except OSError as e:
            print(f"[错误] 读取M3编码器失败: {e}")
            return None

    # 3. 通过PWM控制M3电机（其他电机保持为0）
    # 注意：根据文档，I2C控制必须一次性写入4个电机的数据
    def set_m3_pwm(pwm_value):
        # 限制PWM范围: -3600 到 3600 (文档规定)
        if pwm_value > 3600:
            pwm_value = 3600
            print(f"[警告] PWM值超限，已限制为: {pwm_value}")
        elif pwm_value < -3600:
            pwm_value = -3600
            print(f"[警告] PWM值超限，已限制为: {pwm_value}")
        
        # 转换为大端模式的int16_t数组（必须包含4个电机数据）
        # 数据结构：[M1高, M1低, M2高, M2低, M3高, M3低, M4高, M4低]
        data_bytes = bytearray()
        data_bytes.extend(struct.pack('>h', 0))   # M1 PWM = 0
        data_bytes.extend(struct.pack('>h', 0))   # M2 PWM = 0
        data_bytes.extend(struct.pack('>h', int(pwm_value))) # M3 PWM = 设定值
        data_bytes.extend(struct.pack('>h', 0))   # M4 PWM = 0
        
        try:
            # 写入PWM控制寄存器 (0x07)
            bus.write_i2c_block_data(DEVICE_ADDR, REG_PWM_CONTROL, list(data_bytes))
            print(f"[控制] 设置M3 PWM值: {pwm_value} (范围: -3600 ~ 3600)")
            return True
        except OSError as e:
            print(f"[错误] I2C 写入失败: {e}")
            return False

    # 4. 通过速度控制M3电机（仅对带编码器的电机有效）
    def set_m3_speed(speed_value):
        # 限制速度范围: -1000 到 1000 (文档规定)
        if speed_value > 1000:
            speed_value = 1000
            print(f"[警告] 速度值超限，已限制为: {speed_value}")
        elif speed_value < -1000:
            speed_value = -1000
            print(f"[警告] 速度值超限，已限制为: {speed_value}")
        
        # 转换为大端模式的int16_t数组（必须包含4个电机数据）
        data_bytes = bytearray()
        data_bytes.extend(struct.pack('>h', 0))   # M1 速度 = 0
        data_bytes.extend(struct.pack('>h', 0))   # M2 速度 = 0
        data_bytes.extend(struct.pack('>h', int(speed_value))) # M3 速度 = 设定值
        data_bytes.extend(struct.pack('>h', 0))   # M4 速度 = 0
        
        try:
            # 写入速度控制寄存器 (0x06)
            bus.write_i2c_block_data(DEVICE_ADDR, REG_SPEED_CONTROL, list(data_bytes))
            print(f"[控制] 设置M3速度值: {speed_value} (范围: -1000 ~ 1000)")
            print("       *注意：速度指令仅对带编码器的电机有效")
            return True
        except OSError as e:
            print(f"[错误] I2C 写入失败: {e}")
            return False

    # 5. （可选）配置电机参数（全局配置，影响所有电机）
    def config_motor_params():
        try:
            print("\n=== 配置电机参数（可选，全局生效）===")
            # 注意：此配置会同时应用到所有电机。如果其他电机类型不同，请谨慎使用。
            # 1. 配置电机类型（根据您的电机选择）
            # 1:520电机, 2:310电机, 3:TT电机(带编码器), 4:TT电机(不带编码器)
            motor_type = 1  # 请根据实际电机修改
            print(f"1. 配置电机类型为: {motor_type}")
            bus.write_byte_data(DEVICE_ADDR, REG_MOTOR_TYPE, motor_type)
            time.sleep(0.1)
            
            # 2. 配置死区（默认1600，可根据需要调整）
            deadzone_value = 1600
            print(f"2. 配置死区值为: {deadzone_value}")
            deadzone_data = struct.pack('>h', deadzone_value)
            bus.write_i2c_block_data(DEVICE_ADDR, REG_DEADZONE, list(deadzone_data))
            time.sleep(0.1)
            
            # 3. 配置磁环线数（带编码器电机必须正确设置）
            magnetic_lines = 11  # 文档默认值，请根据电机参数表修改
            print(f"3. 配置磁环线数为: {magnetic_lines}")
            magnetic_data = struct.pack('>H', magnetic_lines)
            bus.write_i2c_block_data(DEVICE_ADDR, REG_MAGNETIC_LINES, list(magnetic_data))
            time.sleep(0.1)
            
            # 4. 配置减速比（带编码器电机必须正确设置）
            reduction_ratio = 30  # 文档默认值，请根据电机参数表修改
            print(f"4. 配置减速比为: {reduction_ratio}")
            reduction_data = struct.pack('>H', reduction_ratio)
            bus.write_i2c_block_data(DEVICE_ADDR, REG_REDUCTION_RATIO, list(reduction_data))
            time.sleep(0.1)
            
            print("[完成] 电机参数配置完成！")
            return True
        except OSError as e:
            print(f"[错误] 电机配置失败: {e}")
            return False

    # ============================
    # 开始测试流程
    # ============================
    try:
        print("="*50)
        print("M3电机（右上轮）测试开始")
        print("="*50)
        
        # 步骤1: 读取电池电压，确认I2C通信正常
        print("\n[步骤1] 读取电池电压...")
        voltage = read_battery_voltage()
        if voltage is None:
            print("[警告] 无法读取电池电量，请检查接线和地址。尝试继续测试...")
        elif voltage < 6.0:
            print(f"[警告] 电池电压({voltage:.2f}V)较低，可能影响电机性能。")
        
        # 步骤2: （可选）配置电机参数
        # config_motor_params() # 取消注释以启用配置
        
        # 步骤3: 读取M3编码器初始值
        print("\n[步骤2] 读取M3编码器初始值...")
        initial_encoder = read_m3_encoder()
        
        # 步骤4: PWM控制测试 - 正向转动
        print("\n[步骤3] PWM控制测试 - M3正向转动")
        print("        预计M3电机（右上轮）会正向转动")
        if set_m3_pwm(1500):
            print("        PWM指令发送成功，等待3秒...")
            time.sleep(3)
            # 读取编码器值查看变化
            read_m3_encoder()
        else:
            print("        PWM控制发送失败")
        
        # 步骤5: PWM控制测试 - 反向转动（倒退）
        print("\n[步骤4] PWM控制测试 - M3反向转动（倒退）")
        print("        预计M3电机（右上轮）会反向转动")
        if set_m3_pwm(-1500):
            print("        PWM指令发送成功，等待3秒...")
            time.sleep(3)
            read_m3_encoder()
        
        # 步骤6: 停止电机（PWM模式）
        print("\n[步骤5] 停止M3电机（PWM置0）...")
        set_m3_pwm(0)
        print("        电机应停止转动")
        time.sleep(1)
        
        # 步骤7: 速度控制测试（仅当电机带编码器时有效）
        print("\n[步骤6] 速度控制测试 - M3以中速转动")
        print("        *此指令仅对带编码器的电机有效*")
        if set_m3_speed(400):
            print("        速度指令发送成功，等待3秒...")
            time.sleep(3)
            read_m3_encoder()
        
        # 步骤8: 速度控制测试 - 反向
        print("\n[步骤7] 速度控制测试 - M3反向转动")
        if set_m3_speed(-400):
            print("        速度指令发送成功，等待3秒...")
            time.sleep(3)
            read_m3_encoder()
        
        # 步骤9: 停止电机（速度模式）
        print("\n[步骤8] 停止M3电机（速度置0）...")
        set_m3_speed(0)
        print("        电机应停止转动（PID锁止状态）")
        time.sleep(1)
        
        # 步骤10: 最终编码器读数
        print("\n[步骤9] 读取M3编码器最终值...")
        final_encoder = read_m3_encoder()
        if initial_encoder is not None and final_encoder is not None:
            print(f"        编码器变化量: {final_encoder - initial_encoder}")
        
        print("\n" + "="*50)
        print("M3电机测试完成！")
        print("="*50)
        
    except KeyboardInterrupt:
        print("\n\n[用户中断] 正在停止电机并退出...")
        set_m3_pwm(0)  # 紧急停止
        set_m3_speed(0)
    except Exception as e:
        print(f"\n[异常] 测试过程中发生错误: {e}")
    finally:
        # 确保电机停止
        try:
            set_m3_pwm(0)
            set_m3_speed(0)
            bus.close()
            print("[清理] I2C总线已关闭，电机已停止。")
        except:
            pass

if __name__ == "__main__":
    main()
