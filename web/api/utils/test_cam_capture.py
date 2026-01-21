
import sys
import os
from pathlib import Path

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parents[0]
sys.path.insert(0, str(PROJECT_ROOT))

from src.support.driver.camera_driver import CameraDriver
import cv2

def test_camera():
    print("Initializing CameraDriver...")
    cam = CameraDriver()
    
    print("Attempting to get frame...")
    frame = cam.get_frame()
    
    if frame is not None:
        print(f"Frame captured successfully. Shape: {frame.shape}")
        cv2.imwrite("test_capture.jpg", frame)
        print("Saved to test_capture.jpg")
    else:
        print("Failed to capture frame")
        
    cam.close()

if __name__ == "__main__":
    test_camera()
