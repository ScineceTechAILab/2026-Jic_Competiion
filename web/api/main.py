import os
import yaml
import time
import threading
import math

from pathlib import Path
from fastapi import FastAPI, HTTPException, Body
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import StreamingResponse
from pydantic import BaseModel

from src.support.logger import get_logger

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    ROS_AVAILABLE = True
except Exception:
    rclpy = None
    Node = object
    qos_profile_sensor_data = None
    LaserScan = None
    ROS_AVAILABLE = False

from src.support.driver.chassis_driver import ChassisDriver
from src.support.driver.imu_driver import IMUDriver
from src.support.driver.camera_driver import CameraDriver

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parents[2]

logger = get_logger("web_api")

app = FastAPI()

# Enable CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


CONFIG_PATH = PROJECT_ROOT / 'config/chasis_params.yaml'
driver = None
imu_driver = None
camera_driver = None
lidar_bridge = None

def get_driver():
    global driver
    if driver is None:
        try:
            driver = ChassisDriver()
        except Exception as e:
            logger.error(f"Failed to initialize chassis driver: {e}")
    return driver

def get_imu_driver():
    global imu_driver
    if imu_driver is None:
        try:
            imu_driver = IMUDriver()
        except Exception as e:
            logger.error(f"Failed to initialize IMU driver: {e}")
    return imu_driver

def get_camera_driver():
    global camera_driver
    if camera_driver is None:
        try:
            camera_driver = CameraDriver()
        except Exception as e:
            logger.error(f"Failed to initialize Camera driver: {e}")
    return camera_driver

class SpeedCorrection(BaseModel):
    left_scale: float
    right_scale: float

class ControlCommand(BaseModel):
    action: str  # "rotate_cw", "rotate_ccw", "stop"
    linear_speed: float = 0.5
    angular_speed: float = 1.5


class RosLidarBridge:
    def __init__(self, topic_name: str = "/lidar_node/scan"):
        self.topic_name = topic_name
        self._lock = threading.Lock()
        self._latest_scan = None
        self._owns_context = False
        self._node = None
        self._spin_thread = None
        self._started = False

        if not ROS_AVAILABLE:
            logger.error("ROS2 packages are not available in this environment")
            return

        try:
            context_ready = rclpy.ok()
        except Exception:
            context_ready = False

        if not context_ready:
            rclpy.init(args=None)
            self._owns_context = True

        self._node = rclpy.create_node('web_lidar_bridge')
        self._node.create_subscription(
            LaserScan,
            self.topic_name,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self._spin_thread = threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True)
        self._spin_thread.start()
        self._started = True
        logger.info(f"Subscribed to LiDAR topic: {self.topic_name}")

    def _on_scan(self, msg):
        # JSON does not support NaN/Infinity, so convert invalid ranges to null.
        ranges = []
        for value in msg.ranges:
            if value is None:
                ranges.append(None)
                continue

            f_value = float(value)
            ranges.append(f_value if math.isfinite(f_value) else None)

        intensities = [float(value) for value in msg.intensities] if msg.intensities else []
        stamp = msg.header.stamp
        timestamp = float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

        with self._lock:
            self._latest_scan = {
                'timestamp': timestamp,
                'frame_id': msg.header.frame_id,
                'ranges': ranges,
                'angle_min': float(msg.angle_min),
                'angle_max': float(msg.angle_max),
                'angle_increment': float(msg.angle_increment),
                'time_increment': float(msg.time_increment),
                'scan_time': float(msg.scan_time),
                'range_min': float(msg.range_min),
                'range_max': float(msg.range_max),
                'intensities': intensities,
            }

    def get_scan(self):
        with self._lock:
            return dict(self._latest_scan) if self._latest_scan else None

    def close(self):
        if not self._started:
            return

        if self._node is not None:
            try:
                self._node.destroy_node()
            except Exception as exc:
                logger.warning(f"Failed to destroy LiDAR bridge node cleanly: {exc}")
            self._node = None

        if self._owns_context and ROS_AVAILABLE and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as exc:
                logger.warning(f"Failed to shutdown ROS2 cleanly: {exc}")


def get_lidar_bridge():
    global lidar_bridge
    if lidar_bridge is None:
        topic_name = os.getenv('WEB_LIDAR_TOPIC', '/lidar_node/scan')
        lidar_bridge = RosLidarBridge(topic_name=topic_name)
    return lidar_bridge

@app.get("/api/config")
async def get_config():
    if CONFIG_PATH.exists():
        with open(CONFIG_PATH, 'r') as f:
            config = yaml.safe_load(f)
            return config.get('speed_correction', {'left_scale': 1.0, 'right_scale': 1.0})
    return {'left_scale': 1.0, 'right_scale': 1.0}

@app.post("/api/config")
async def update_config(correction: SpeedCorrection):
    if not CONFIG_PATH.exists():
        raise HTTPException(status_code=404, detail="Config file not found")
    
    try:
        with open(CONFIG_PATH, 'r') as f:
            config = yaml.safe_load(f) or {}
        
        config['speed_correction'] = {
            'left_scale': correction.left_scale,
            'right_scale': correction.right_scale
        }
        
        with open(CONFIG_PATH, 'w') as f:
            yaml.dump(config, f)
            
        # Update driver if active
        d = get_driver()
        if d:
            d.reconfig_speed_correction(correction.left_scale, correction.right_scale)
            
        return {"status": "success", "config": config['speed_correction']}
    except Exception as e:
        logger.error(f"Failed to update config: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/control")
async def control_chassis(cmd: ControlCommand):
    d = get_driver()
    if not d:
        raise HTTPException(status_code=500, detail="Chassis driver not initialized")
    
    try:
        # Re-apply current config to ensure scales are up to date
        with open(CONFIG_PATH, 'r') as f:
            config = yaml.safe_load(f)
            sc = config.get('speed_correction', {})
            d.reconfig_speed_correction(sc.get('left_scale', 1.0), sc.get('right_scale', 1.0))

        if cmd.action == "rotate_cw":
            # Clockwise: Left Forward, Right Backward
            # Angular < 0
            d.rotate(-abs(cmd.angular_speed))
        elif cmd.action == "rotate_ccw":
            # Counter-Clockwise: Left Backward, Right Forward
            # Angular > 0
            d.rotate(abs(cmd.angular_speed))
        elif cmd.action == "move_forward":
            # Move Forward: Linear > 0
            d.move(abs(cmd.linear_speed))
        elif cmd.action == "move_backward":
            # Move Backward: Linear < 0
            d.move(-abs(cmd.linear_speed))
        elif cmd.action == "stop":
            d.stop()
        else:
            return {"status": "error", "message": "Unknown action"}
            
        return {"status": "success", "action": cmd.action}
    except Exception as e:
        logger.error(f"Control failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/battery")
async def get_battery_info():
    d = get_driver()
    if not d:
        raise HTTPException(status_code=500, detail="Chassis driver not initialized")
    try:
        voltage = d.battery_voltage()
        return {"voltage": voltage}
    except Exception as e:
        logger.error(f"Failed to get battery info: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/chassis/speed")
async def get_wheel_speed():
    d = get_driver()
    if not d:
        raise HTTPException(status_code=500, detail="Chassis driver not initialized")
    try:
        left_speed, right_speed = d.wheel_speed()
        return {"left_speed": left_speed, "right_speed": right_speed}
    except Exception as e:
        logger.error(f"Failed to get wheel speed: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/imu")
async def get_imu_data():
    d = get_imu_driver()
    if not d:
        raise HTTPException(status_code=500, detail="IMU driver not initialized")
    try:
        return d.get_data()
    except Exception as e:
        logger.error(f"Failed to get IMU data: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/lidar")
async def get_lidar_data():
    bridge = get_lidar_bridge()
    if not bridge or not ROS_AVAILABLE:
        raise HTTPException(status_code=503, detail="ROS2 LiDAR bridge is not available")

    scan = bridge.get_scan()
    if not scan:
        raise HTTPException(status_code=503, detail="No LiDAR scan received yet")

    return scan

@app.get("/api/camera/stream")
async def video_feed():
    def generate():
        cam = get_camera_driver()
        if not cam:
            return
        
        while True:
            frame = cam.get_jpeg_frame()
            if frame:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            else:
                time.sleep(0.1) # Wait if no frame
            time.sleep(0.01) # Small delay to prevent CPU hogging

    return StreamingResponse(generate(), media_type="multipart/x-mixed-replace; boundary=frame")

# Mount static files
app.mount("/", StaticFiles(directory=str(PROJECT_ROOT / "web/public"), html=True), name="static")


@app.on_event("shutdown")
def shutdown_lidar_bridge():
    global lidar_bridge
    if lidar_bridge is not None:
        lidar_bridge.close()
        lidar_bridge = None

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
