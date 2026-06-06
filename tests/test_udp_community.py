import socket
from src.support.logger import get_logger

logger = get_logger(__name__)

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(('0.0.0.0', 11811))
logger.info("正在等待 RDK 的消息...")
while True:
    data, addr = s.recvfrom(1024)
    try:
        logger.info(f"来自 {addr} 的消息: {data.decode('utf-8')}")
    except UnicodeDecodeError:
        logger.warning(f"来自 {addr} 的消息 (非 UTF-8): {data.decode('gbk', errors='ignore')}")


