import time
import sys
import os

# 将项目根目录添加到路径，以便导入 drivers
sys.path.append('/app/jic_competiion')

from src.support.driver.chassis_driver import ChassisDriver

def test_differential_drive():
    print("=== 差速底盘测试程序 ===")
    
    # 初始化驱动
    # 假设轮距 160mm, 轮径 65mm (需要用户根据实际修改)
    chassis = ChassisDriver(wheel_diameter_mm=65, wheel_base_mm=160)
    
    # 检查电池电压
    voltage = chassis.get_battery_voltage()
    print(f"电池电压: {voltage:.2f}V")
    
    # 配置电机参数 (假设使用520电机)
    print("配置电机参数...")
    chassis.configure_motors(motor_type=1, reduction_ratio=30)
    
    # 交互式测试电机映射
    print("\n[Phase 1] 确认电机映射")
    print("请观察哪个轮子在转动...")
    
    # 测试 M1
    print("正在驱动 M1 电机 (正向)...")
    chassis._set_motors_speed([200, 0, 0, 0])
    time.sleep(2)
    chassis.stop()
    m1_role = input("M1 是左轮(L)、右轮(R) 还是其他(O)? [L/R/O]: ").strip().upper()
    
    # 测试 M2
    print("正在驱动 M2 电机 (正向)...")
    chassis._set_motors_speed([0, 200, 0, 0])
    time.sleep(2)
    chassis.stop()
    m2_role = input("M2 是左轮(L)、右轮(R) 还是其他(O)? [L/R/O]: ").strip().upper()

    # 测试 M3 (如果需要)
    # 测试 M4 (如果需要)
    
    # 根据用户输入配置映射
    left_id = 0
    right_id = 0
    
    if m1_role == 'L': left_id = 1
    elif m1_role == 'R': right_id = 1
    
    if m2_role == 'L': left_id = 2
    elif m2_role == 'R': right_id = 2
    
    # 默认值回退
    if left_id == 0: 
        print("未检测到左轮，默认设为 M2")
        left_id = 2
    if right_id == 0:
        print("未检测到右轮，默认设为 M1")
        right_id = 1
        
    print(f"\n配置映射: 左轮=M{left_id}, 右轮=M{right_id}")
    chassis.set_motor_mapping(left_id, right_id)
    
    # 交互式测试方向
    print("\n[Phase 2] 确认电机方向")
    
    print("正在驱动左轮 (正向指令)...")
    speeds = [0, 0, 0, 0]
    speeds[left_id-1] = 200
    chassis._set_motors_speed(speeds)
    time.sleep(2)
    chassis.stop()
    l_dir_input = input("左轮是向前(F)还是向后(B)? [F/B]: ").strip().upper()
    l_dir = 1 if l_dir_input == 'F' else -1
    
    print("正在驱动右轮 (正向指令)...")
    speeds = [0, 0, 0, 0]
    speeds[right_id-1] = 200
    chassis._set_motors_speed(speeds)
    time.sleep(2)
    chassis.stop()
    r_dir_input = input("右轮是向前(F)还是向后(B)? [F/B]: ").strip().upper()
    r_dir = 1 if r_dir_input == 'F' else -1
    
    print(f"方向修正: 左轮={l_dir}, 右轮={r_dir}")
    chassis.set_motor_mapping(left_id, right_id, l_dir, r_dir)
    
    # 运动学测试
    print("\n[Phase 3] 运动学控制测试")
    print("1. 前进 0.2 m/s")
    chassis.set_movement(0.2, 0)
    time.sleep(2)
    chassis.stop()
    time.sleep(1)
    
    print("2. 后退 0.2 m/s")
    chassis.set_movement(-0.2, 0)
    time.sleep(2)
    chassis.stop()
    time.sleep(1)
    
    print("3. 原地左转 1.0 rad/s")
    chassis.set_movement(0, 1.0)
    time.sleep(2)
    chassis.stop()
    time.sleep(1)
    
    print("测试完成!")
    chassis.close()

if __name__ == "__main__":
    try:
        test_differential_drive()
    except KeyboardInterrupt:
        print("Interrupted")
