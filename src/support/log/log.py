import os
import sys
import rclpy.logging

# 设置 ROS 2 日志格式环境变量，尽量匹配原有格式
# 原格式: [2026-02-04 14:40:35] [INFO] [imu_node.py:62] Message
# ROS 2 占位符: {time}, {severity}, {file_name}, {line_number}, {message}
# 注意: ROS 2 的 {time} 默认是秒级时间戳，颜色由 ROS 2 内部处理
os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = "[{time}] [{severity}] [{file_name}:{line_number}] {message}"

def get_logger(name="root"):
    """
    获取推荐的日志记录器。
    该实现已切换为 ROS 2 内置日志系统，以符合项目标准。
    """
    return rclpy.logging.get_logger(name)

class Logger:
    @staticmethod
    def get_logger(name="root"):
        """
        静态方法获取日志记录器，保持全项目调用兼容性。
        """
        return get_logger(name)
