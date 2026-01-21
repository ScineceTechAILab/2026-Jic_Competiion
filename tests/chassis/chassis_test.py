import sys
import os
import time
import logging

# Add project root to sys.path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../'))
sys.path.insert(0, PROJECT_ROOT)

from src.support.driver.chassis_driver import ChassisDriver

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger("ChassisTest")

class ChassisTest:
    def __init__(self):
        try:
            self.driver = ChassisDriver()
            logger.info("ChassisDriver initialized successfully.")
        except Exception as e:
            logger.error(f"Failed to initialize ChassisDriver: {e}")
            sys.exit(1)

    def test_motor_rotate(self):
        """
        Test individual motors to verify wiring and mapping.
        """
        logger.info(">>> TEST: Individual Motor Rotation")
        
        # Test Left Motor
        logger.info("  -> Testing LEFT Motor (Forward) for 2 seconds...")
        speeds = [0, 0, 0, 0]
        # Assuming left_motor_id is 1-based index
        if 1 <= self.driver.left_motor_id <= 4:
            speeds[self.driver.left_motor_id - 1] = 30  # Low speed (raw PWM/Speed value approx)
            # Note: _set_motors_speed takes values -1000 to 1000 (RPM-like or PWM-like depending on mode)
            # Let's use a safe value like 200 (20% PWM if it were PWM, or 200 RPM)
            speeds[self.driver.left_motor_id - 1] = 200 * self.driver.left_motor_dir
        
        self.driver._set_motors_speed(speeds)
        time.sleep(2)
        self.driver.stop()
        time.sleep(0.5)

        # Test Right Motor
        logger.info("  -> Testing RIGHT Motor (Forward) for 2 seconds...")
        speeds = [0, 0, 0, 0]
        if 1 <= self.driver.right_motor_id <= 4:
            speeds[self.driver.right_motor_id - 1] = 200 * self.driver.right_motor_dir
        
        self.driver._set_motors_speed(speeds)
        time.sleep(2)
        self.driver.stop()
        logger.info("  -> Motor Test Complete\n")

    def test_chassis_translate(self):
        """
        Test linear movement (Forward/Backward).
        """
        logger.info(">>> TEST: Chassis Translation (Linear)")
        
        # Forward
        logger.info("  -> Moving FORWARD at 0.3 m/s for 2 seconds...")
        self.driver.move(0.3)
        time.sleep(2)
        self.driver.stop()
        time.sleep(0.5)

        # Backward
        logger.info("  -> Moving BACKWARD at 0.3 m/s for 2 seconds...")
        self.driver.move(-0.3)
        time.sleep(2)
        self.driver.stop()
        logger.info("  -> Translation Test Complete\n")

    def test_chassis_rotate(self):
        """
        Test rotation (CW/CCW).
        """
        logger.info(">>> TEST: Chassis Rotation")
        
        # CCW (Left)
        logger.info("  -> Rotating CCW (Left) at 1.0 rad/s for 2 seconds...")
        self.driver.rotate(1.0)
        time.sleep(2)
        self.driver.stop()
        time.sleep(0.5)

        # CW (Right)
        logger.info("  -> Rotating CW (Right) at 1.0 rad/s for 2 seconds...")
        self.driver.rotate(-1.0)
        time.sleep(2)
        self.driver.stop()
        logger.info("  -> Rotation Test Complete\n")

    def run_all(self):
        try:
            logger.info("Starting Chassis Test Suite...")
            
            # 1. Check Battery
            voltage = self.driver.battery_voltage()
            logger.debug(f"Current Battery Voltage: {voltage:.2f} V")
            if voltage < 9.0:
                logger.warning("Battery voltage might be too low!")

            # 2. Run Tests
            self.test_motor_rotate()
            time.sleep(1)
            
            self.test_chassis_translate()
            time.sleep(1)
            
            self.test_chassis_rotate()
            
            logger.info("All tests finished successfully.")
            
        except KeyboardInterrupt:
            logger.warning("Test interrupted by user.")
        except Exception as e:
            logger.error(f"Test failed with error: {e}")
        finally:
            logger.info("Stopping all motors...")
            self.driver.stop()
            self.driver.close()

if __name__ == "__main__":
    test = ChassisTest()
    test.run_all()
