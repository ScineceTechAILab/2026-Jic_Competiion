import time
import random

class IMUDriver:
    def __init__(self):
        self.connected = True
        
    def get_data(self):
        """
        Return mock IMU data.
        In a real implementation, this would read from the sensor.
        """
        return {
            "timestamp": time.time(),
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "w": 1.0
            },
            "angular_velocity": {
                "x": random.uniform(-0.1, 0.1),
                "y": random.uniform(-0.1, 0.1),
                "z": random.uniform(-0.1, 0.1)
            },
            "linear_acceleration": {
                "x": random.uniform(-0.1, 0.1),
                "y": random.uniform(-0.1, 0.1),
                "z": 9.8 + random.uniform(-0.1, 0.1)
            }
        }
