import serial
import time
import struct

class RPLidarS1Tester:
    """
    思岚S1激光雷达测试类（协议部分为示例，需根据官方文档校准）
    """
    # ====== 【关键】以下协议常量需要根据思岚S1官方协议文档确认和修改 ======
    # 示例命令（基于常见RPLIDAR指令，非S1官方）
    CMD_STOP = b'\xA5\x25'      # 停止扫描
    CMD_RESET = b'\xA5\x40'     # 复位设备
    CMD_SCAN = b'\xA5\x20'      # 开始扫描（标准模式）
    CMD_EXPRESS_SCAN = b'\xA5\x82\x05\x00\x00\x00\x00\x00\x22'  # 开始快速扫描（示例，需确认）
    
    # 数据包描述符（基于常见RPLIDAR，非S1官方）
    DESC_SYNC_BYTE1 = 0xA5
    DESC_SYNC_BYTE2_SCAN = 0x5A  # 扫描数据包同步字节2
    DESC_SYNC_BYTE2_EXPRESS = 0x82  # 快速扫描数据包同步字节2（示例）
    
    # 数据包类型（基于常见RPLIDAR，非S1官方）
    DATA_TYPE_SCAN = 0x81
    DATA_TYPE_EXPRESS_SCAN = 0x82
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000):
        """
        初始化串口连接。
        参数:
            port: 串口设备路径，如 '/dev/ttyUSB0'
            baudrate: 波特率，思岚S1 USB转接器需设置为256000（手册P4）
        """
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0  # 读超时1秒
        )
        self.is_scanning = False
        print(f"Connected to {port} at {baudrate} baud.")
        
    def send_command(self, cmd_bytes):
        """发送命令到雷达"""
        self.ser.write(cmd_bytes)
        time.sleep(0.01)  # 短暂延时，等待设备响应
        
    def start_scan(self, express_mode=False):
        """
        启动雷达扫描。
        参数:
            express_mode: 是否使用快速扫描模式（如果设备支持）
        """
        # 先停止可能正在进行的扫描
        self.stop_scan()
        time.sleep(0.1)
        
        # 发送开始扫描命令
        if express_mode:
            self.send_command(self.CMD_EXPRESS_SCAN)
            print("Express scan mode started.")
        else:
            self.send_command(self.CMD_SCAN)
            print("Standard scan mode started.")
            
        self.is_scanning = True
        
    def stop_scan(self):
        """停止雷达扫描"""
        if self.is_scanning:
            self.send_command(self.CMD_STOP)
            self.is_scanning = False
            print("Scan stopped.")
            
    def reset(self):
        """复位雷达设备"""
        self.send_command(self.CMD_RESET)
        self.is_scanning = False
        print("Device reset.")
        time.sleep(0.5)  # 复位后需要较长等待时间
        
    def read_and_parse_data(self, duration_seconds=30):
        """
        读取并解析雷达数据。
        参数:
            duration_seconds: 测试持续时间（秒）
        """
        if not self.is_scanning:
            print("Warning: Radar is not in scanning mode. Starting standard scan...")
            self.start_scan(express_mode=False)
            
        print(f"Starting data collection for {duration_seconds} seconds...")
        print("Press Ctrl+C to stop early.")
        
        start_time = time.time()
        packet_count = 0
        point_count = 0
        
        try:
            while time.time() - start_time < duration_seconds:
                # 1. 读取数据
                if self.ser.in_waiting >= 5:  # 至少需要读取描述符头
                    # 读取一个数据包描述符（常见RPLIDAR为5字节）
                    descriptor = self.ser.read(5)
                    
                    # 2. 检查同步头
                    if descriptor[0] != self.DESC_SYNC_BYTE1:
                        # 同步字节错误，尝试重新对齐
                        self._resync_buffer()
                        continue
                        
                    # 3. 解析描述符
                    data_length = descriptor[1]  # 数据长度（低字节）
                    send_mode = descriptor[2]    # 发送模式
                    data_type = descriptor[3]    # 数据类型
                    data_length |= descriptor[4] << 8  # 数据长度（高字节）
                    
                    # 4. 根据数据类型读取和解析数据
                    if data_type == self.DATA_TYPE_SCAN:
                        # 标准扫描数据包
                        if self.ser.in_waiting >= data_length:
                            raw_data = self.ser.read(data_length)
                            measurements = self._parse_scan_data(raw_data)
                            packet_count += 1
                            point_count += len(measurements)
                            self._print_measurements(measurements)
                    elif data_type == self.DATA_TYPE_EXPRESS_SCAN:
                        # 快速扫描数据包（需要不同的解析方式）
                        if self.ser.in_waiting >= data_length:
                            raw_data = self.ser.read(data_length)
                            measurements = self._parse_express_scan_data(raw_data)
                            packet_count += 1
                            point_count += len(measurements)
                            self._print_measurements(measurements)
                    else:
                        # 未知数据类型，跳过
                        if self.ser.in_waiting >= data_length:
                            self.ser.read(data_length)  # 丢弃数据
                            
                time.sleep(0.001)  # 短暂休眠，避免CPU占用过高
                
        except KeyboardInterrupt:
            print("\nData collection interrupted by user.")
        finally:
            elapsed = time.time() - start_time
            print(f"\n--- Test Summary ---")
            print(f"Duration: {elapsed:.1f} seconds")
            print(f"Packets received: {packet_count}")
            print(f"Total points parsed: {point_count}")
            if elapsed > 0:
                print(f"Average rate: {point_count/elapsed:.1f} points/sec")
                
    def _resync_buffer(self):
        """尝试重新同步数据流（查找同步字节）"""
        print("Attempting to resync data stream...")
        while self.ser.in_waiting > 0:
            byte = self.ser.read(1)
            if byte[0] == self.DESC_SYNC_BYTE1:
                # 找到可能的同步头，尝试读取后续字节验证
                if self.ser.in_waiting >= 4:
                    next_bytes = self.ser.read(4)
                    if next_bytes[0] == self.DESC_SYNC_BYTE2_SCAN or \
                       next_bytes[0] == self.DESC_SYNC_BYTE2_EXPRESS:
                        # 验证成功，将指针回退以便主循环重新读取完整描述符
                        self.ser.read(self.ser.in_waiting)  # 清空缓冲区
                        return
        print("Resync failed. Buffer cleared.")
        
    def _parse_scan_data(self, raw_data):
        """
        解析标准扫描数据包。
        注意：此解析函数基于常见RPLIDAR格式，必须根据思岚S1官方协议修改！
        参数:
            raw_data: 原始数据字节
        返回:
            measurements: 列表，每个元素为(角度_度, 距离_米, 质量)
        """
        measurements = []
        # 示例解析：假设每个数据点占5字节（常见格式）
        # 实际格式需参考思岚S1协议文档
        point_size = 5
        num_points = len(raw_data) // point_size
        
        for i in range(num_points):
            start_idx = i * point_size
            point_data = raw_data[start_idx:start_idx + point_size]
            
            # 示例解析（需要根据实际协议调整！）
            # 假设格式：[质量(1字节)][距离低字节(1字节)][距离高字节(1字节)][角度低字节(1字节)][角度高字节(1字节)]
            quality = point_data
            distance_raw = (point_data[2] << 8) | point_data[1]  # 小端格式
            angle_raw = (point_data[4] << 8) | point_data[3]     # 小端格式
            
            # 单位转换（需要根据协议文档确认转换系数！）
            # 常见转换：角度 * 0.01 = 度，距离 * 0.001 = 米
            angle_deg = (angle_raw >> 1) / 64.0  # 示例转换
            distance_m = distance_raw / 4000.0   # 示例转换
            
            measurements.append((angle_deg, distance_m, quality))
            
        return measurements
        
    def _parse_express_scan_data(self, raw_data):
        """
        解析快速扫描数据包。
        注意：此函数为示例，必须根据思岚S1官方协议实现！
        """
        measurements = []
        # 快速扫描模式有更紧凑的数据格式
        # 需要根据官方文档实现具体解析逻辑
        print("Express scan data parsing not implemented. Check official protocol.")
        return measurements
        
    def _print_measurements(self, measurements, max_points=5):
        """打印测量结果（限制数量以避免输出过多）"""
        if not measurements:
            return
            
        print(f"\n--- New packet with {len(measurements)} points ---")
        for i, (angle, distance, quality) in enumerate(measurements[:max_points]):
            print(f"Point {i+1}: Angle={angle:6.2f}°, Distance={distance:6.3f}m, Quality={quality:3d}")
        if len(measurements) > max_points:
            print(f"... and {len(measurements) - max_points} more points")
            
    def close(self):
        """关闭串口连接"""
        self.stop_scan()
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial port closed.")
            
    def run_test_sequence(self):
        """运行完整的测试序列"""
        try:
            # 1. 复位设备（可选）
            self.reset()
            
            # 2. 获取设备信息（需要实现相应命令）
            # self.get_device_info()
            
            # 3. 开始扫描（标准模式）
            self.start_scan(express_mode=False)
            time.sleep(1.0)  # 等待电机稳定
            
            # 4. 读取并解析数据
            self.read_and_parse_data(duration_seconds=10)
            
            # 5. 切换模式测试（如果支持）
            # self.stop_scan()
            # time.sleep(0.5)
            # self.start_scan(express_mode=True)
            # self.read_and_parse_data(duration_seconds=5)
            
        except Exception as e:
            print(f"Error during test: {e}")
        finally:
            self.close()


def main():
    """
    主函数：查找可用串口并运行测试
    """
    # 可能的串口设备列表（Linux系统）
    possible_ports = [
        '/dev/ttyUSB0',  # USB转串口适配器
        '/dev/ttyUSB1',
        '/dev/ttyACM0',  # CDC ACM设备（如某些USB转串口）
        '/dev/ttyS0',    # 原生串口（通常为调试口）
        '/dev/ttyS1',
        '/dev/ttyS2',    # RDK X5可能的UART3设备节点
        '/dev/ttyS3',
    ]
    
    # 尝试连接第一个可用的端口
    connected = False
    lidar = None
    
    for port in possible_ports:
        try:
            print(f"Trying to connect to {port}...")
            lidar = RPLidarS1Tester(port=port, baudrate=256000)
            connected = True
            print(f"Successfully connected to {port}")
            break
        except (serial.SerialException, OSError) as e:
            print(f"Failed to connect to {port}: {e}")
            continue
    
    if not connected:
        print("Error: Could not connect to any serial port.")
        print("Please check:")
        print("1. Radar is powered and connected")
        print("2. USB转接器拨码开关设置为256000波特率（思岚S1手册P4）")
        print("3. Run 'ls /dev/tty*' to find the correct port")
        print("4. User has permission to access the port (e.g., in dialout group)")
        return
    
    # 运行测试
    try:
        lidar.run_test_sequence()
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
        if lidar:
            lidar.close()


if __name__ == "__main__":
    main()
