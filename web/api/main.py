import sys
import os
import yaml
from pathlib import Path
from fastapi import FastAPI, HTTPException, Body
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from src.support.driver.chassis_driver import ChassisDriver
from src.support.log import get_logger

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

def get_driver():
    global driver
    if driver is None:
        try:
            driver = ChassisDriver()
            # Default mapping, should match config
            driver.set_motor_mapping(left_id=2, right_id=3)
        except Exception as e:
            logger.error(f"Failed to initialize driver: {e}")
    return driver

class SpeedCorrection(BaseModel):
    left_scale: float
    right_scale: float

class ControlCommand(BaseModel):
    action: str  # "rotate_cw", "rotate_ccw", "stop"

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
            d.set_speed_correction(correction.left_scale, correction.right_scale)
            
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
            d.set_speed_correction(sc.get('left_scale', 1.0), sc.get('right_scale', 1.0))

        if cmd.action == "rotate_cw":
            # Clockwise: Left Forward, Right Backward
            # Linear=0, Angular < 0
            d.set_movement(0, -1.5)
        elif cmd.action == "rotate_ccw":
            # Counter-Clockwise: Left Backward, Right Forward
            # Linear=0, Angular > 0
            d.set_movement(0, 1.5)
        elif cmd.action == "stop":
            d.stop()
        else:
            raise HTTPException(status_code=400, detail="Invalid action")
            
        return {"status": "executed", "action": cmd.action}
    except Exception as e:
        logger.error(f"Control error: {e}")
        d.stop() # Safety stop
        raise HTTPException(status_code=500, detail=str(e))

# Mount static files
app.mount("/", StaticFiles(directory=str(PROJECT_ROOT / "web/public"), html=True), name="static")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
