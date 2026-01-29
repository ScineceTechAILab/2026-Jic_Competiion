#!/usr/bin/env python3
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Add project root to path to import support modules
# Assuming structure: /app/jic_competiion/src/execution/imu_control/imu_control/imu_node.py
# Root is up 4 levels
PROJECT_ROOT = Path(__file__).parents[4]
sys.path.insert(0, str(PROJECT_ROOT))

try:
    from src.support.driver.imu_driver import IMUDriver
    from src.support.log import get_logger
except ImportError as e:
    # If running in a way where src works (e.g. tests)
    print(f"Import Error: {e}. Trying alternate import...")
    try:
        from support.driver.imu_driver import IMUDriver
        from support.log import get_logger
    except ImportError:
        # Fallback for when installed in colcon environment and src.support is not in path?
        # In a real deployment, shared code should be a library package.
        # For now, we rely on the path insertion above.
        raise

logger = get_logger(__name__)

class ImuNode(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('pub_rate', 100.0) # Hz
        
        self.frame_id = self.get_parameter('frame_id').value
        self.rate = self.get_parameter('pub_rate').value
        
        try:
            self.driver = IMUDriver()
            if not self.driver.connected:
                logger.error("IMU Driver reports not connected.")
                pass # We continue, maybe it's intermittent, or we want to retry in loop
            else:
                logger.info("IMU Driver Initialized Successfully")
        except Exception as e:
            logger.error(f"Failed to initialize IMU Driver: {e}")
            self.destroy_node()
            return

        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)
        logger.info("IMU Node Started")

    def timer_callback(self):
        data = self.driver.get_data()
        
        if data is None:
            # logger.warning("Failed to read IMU data") # too verbose if fails often
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        
        # Orientation
        msg.orientation.x = float(data['orientation']['x'])
        msg.orientation.y = float(data['orientation']['y'])
        msg.orientation.z = float(data['orientation']['z'])
        msg.orientation.w = float(data['orientation']['w'])
        
        # Angular Velocity
        msg.angular_velocity.x = float(data['angular_velocity']['x'])
        msg.angular_velocity.y = float(data['angular_velocity']['y'])
        msg.angular_velocity.z = float(data['angular_velocity']['z'])
        
        # Linear Acceleration
        msg.linear_acceleration.x = float(data['linear_acceleration']['x'])
        msg.linear_acceleration.y = float(data['linear_acceleration']['y'])
        msg.linear_acceleration.z = float(data['linear_acceleration']['z'])
        
        # Covariance matrices - TODO: Load from config or calibration
        # Setting to 0 essentially says "unknown" or "perfect" depending on interpretation, 
        # usually 0 on diagonal for unknown is safer or -1 for first element
        msg.orientation_covariance = [0.0] * 9
        msg.angular_velocity_covariance = [0.0] * 9
        msg.linear_acceleration_covariance = [0.0] * 9
        
        self.imu_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
