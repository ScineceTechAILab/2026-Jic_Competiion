#!/usr/bin/env python3
"""
简化的VNC摄像头显示脚本
更稳定的版本，适合在VNC环境中使用
"""

import cv2
import numpy as np
import time
from datetime import datetime
import sys

def display_camera_stream(camera_index=0, width=640, height=480, fps=30):
    """
    显示摄像头实时视频流
    
    Args:
        camera_index: 摄像头索引 (0, 1, 等)
        width: 视频宽度
        height: 视频高度
        fps: 帧率
    """
    print(f"\n{'='*60}")
    print(f"{'VNC摄像头显示 - 简化版':^60}")
    print(f"{'='*60}\n")
    
    # 初始化摄像头
    print(f"[*] 初始化摄像头 {camera_index}...")
    cap = cv2.VideoCapture(camera_index)
    
    if not cap.isOpened():
        print(f"[✗] 无法打开摄像头 {camera_index}")
        return False
    
    # 设置摄像头参数
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    
    # 读取实际参数
    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = int(cap.get(cv2.CAP_PROP_FPS))
    
    print(f"[✓] 摄像头初始化成功")
    print(f"    分辨率: {actual_width} x {actual_height}")
    print(f"    帧率: {actual_fps} FPS\n")
    
    print("[*] 显示摄像头画面...")
    print("[提示] 按下列按键进行操作:")
    print("    - 'q': 退出")
    print("    - 's': 保存当前帧")
    print("    - 'r': 开始/停止录制")
    print("    - 'p': 暂停/继续播放")
    print("    - 'b': 切换黑白模式")
    print("    - 'f': 切换全屏模式\n")
    
    frame_count = 0
    start_time = time.time()
    fps_counter = 0
    current_fps = 0
    
    video_writer = None
    is_recording = False
    is_paused = False
    use_grayscale = False
    fullscreen = False
    
    window_name = "Camera Stream"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, actual_width, actual_height)
    
    save_counter = 0
    
    try:
        while True:
            if not is_paused:
                ret, frame = cap.read()
                
                if not ret:
                    print("[✗] 无法读取摄像头数据")
                    break
                
                # 计算FPS
                frame_count += 1
                fps_counter += 1
                current_time = time.time()
                
                if current_time - start_time >= 1.0:
                    current_fps = fps_counter
                    fps_counter = 0
                    start_time = current_time
                
                # 应用黑白模式
                if use_grayscale:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                
                # 添加信息文本
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                cv2.putText(frame, f"Time: {timestamp}", (10, 25),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                cv2.putText(frame, f"FPS: {current_fps}", (10, 50),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                cv2.putText(frame, f"Frames: {frame_count}", (10, 75),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                
                # 显示状态
                status_text = ""
                if is_paused:
                    status_text += "[PAUSED] "
                if is_recording:
                    status_text += "[RECORDING] "
                if use_grayscale:
                    status_text += "[B&W]"
                
                if status_text:
                    cv2.putText(frame, status_text, (10, actual_height - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
                
                # 录制视频
                if is_recording and video_writer is not None:
                    video_writer.write(frame)
                
                # 显示帧
                cv2.imshow(window_name, frame)
            
            # 处理键盘输入
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("\n[*] 收到退出命令...")
                break
            
            elif key == ord('s'):
                save_counter += 1
                filename = f"camera_frame_{save_counter}_{int(time.time())}.jpg"
                cv2.imwrite(filename, frame)
                print(f"[✓] 已保存帧: {filename}")
            
            elif key == ord('r'):
                if not is_recording:
                    # 开始录制
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    video_filename = f"camera_video_{timestamp}.avi"
                    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
                    video_writer = cv2.VideoWriter(video_filename, fourcc, actual_fps, 
                                                  (actual_width, actual_height))
                    is_recording = True
                    print(f"[▶] 开始录制: {video_filename}")
                else:
                    # 停止录制
                    if video_writer is not None:
                        video_writer.release()
                        video_writer = None
                    is_recording = False
                    print(f"[⏹] 录制已停止")
            
            elif key == ord('p'):
                is_paused = not is_paused
                if is_paused:
                    print("[⏸] 视频已暂停")
                else:
                    print("[▶] 视频已继续")
            
            elif key == ord('b'):
                use_grayscale = not use_grayscale
                if use_grayscale:
                    print("[B&W] 已切换为黑白模式")
                else:
                    print("[COLOR] 已切换为彩色模式")
            
            elif key == ord('f'):
                fullscreen = not fullscreen
                if fullscreen:
                    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, 
                                        cv2.WINDOW_FULLSCREEN)
                    print("[全屏] 已进入全屏模式")
                else:
                    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, 
                                        cv2.WINDOW_NORMAL)
                    print("[窗口] 已退出全屏模式")
    
    except KeyboardInterrupt:
        print("\n[*] 收到中断信号...")
    
    finally:
        # 清理资源
        print("\n[*] 正在释放资源...")
        
        if is_recording and video_writer is not None:
            video_writer.release()
        
        cap.release()
        cv2.destroyAllWindows()
        
        print(f"[✓] 已释放所有资源")
        print(f"[统计] 总共捕获 {frame_count} 帧")
        print(f"[统计] 平均FPS: {frame_count / (time.time() - (start_time - 1.0)):.1f}\n")
        
        return True


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="VNC摄像头显示工具 - 简化版")
    parser.add_argument('--camera', type=int, default=0, help='摄像头索引')
    parser.add_argument('--width', type=int, default=640, help='视频宽度')
    parser.add_argument('--height', type=int, default=480, help='视频高度')
    parser.add_argument('--fps', type=int, default=30, help='目标帧率')
    
    args = parser.parse_args()
    
    success = display_camera_stream(
        camera_index=args.camera,
        width=args.width,
        height=args.height,
        fps=args.fps
    )
    
    sys.exit(0 if success else 1)
