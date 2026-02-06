import socket
import time

# 目标配置
TARGET_IP = "10.70.90.197"
TARGET_PORT = 11811
INTERVAL = 1.0  # 发送间隔 (秒)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"🚀 开始循环发送 UDP 消息到 {TARGET_IP}:{TARGET_PORT} (间隔: {INTERVAL}s)...")
print("按 Ctrl+C 停止发送")

count = 0
try:
    while True:
        count += 1
        msg = f"HELLO_FROM_RDK [Count: {count}]".encode()
        s.sendto(msg, (TARGET_IP, TARGET_PORT))
        print(f"[{time.strftime('%H:%M:%S')}] 第 {count} 次消息已发出")
        time.sleep(INTERVAL)
except KeyboardInterrupt:
    print("\n⏹ 发送已由用户停止")
finally:
    s.close()
