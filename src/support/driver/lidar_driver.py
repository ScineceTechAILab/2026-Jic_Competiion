import time
import math
import random

class LidarDriver:
    def __init__(self):
        self.connected = True
        
    def get_scan(self):
        """
        Return mock LiDAR scan data.
        Returns a list of ranges in meters for 360 degrees.
        """
        ranges = []
        for angle in range(360):
            # Simulate a room: square-ish
            rad = math.radians(angle)
            # Simple wall simulation (e.g., 5m x 5m room)
            # Distance to wall at angle theta: d = min(|x_wall/cos(theta)|, |y_wall/sin(theta)|)
            # x_wall = 2.5, y_wall = 2.5
            
            # Avoid division by zero
            c = math.cos(rad)
            s = math.sin(rad)
            
            dist_x = 2.5 / abs(c) if abs(c) > 1e-3 else 100.0
            dist_y = 2.5 / abs(s) if abs(s) > 1e-3 else 100.0
            
            dist = min(dist_x, dist_y)
            
            # Add some noise
            dist += random.uniform(-0.05, 0.05)
            
            ranges.append(dist)
            
        return {
            "timestamp": time.time(),
            "ranges": ranges,
            "angle_min": 0,
            "angle_max": 2 * math.pi,
            "angle_increment": 2 * math.pi / 360,
            "range_min": 0.1,
            "range_max": 12.0
        }
