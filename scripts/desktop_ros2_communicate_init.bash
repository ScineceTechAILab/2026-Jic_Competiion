#!/bin/bash
# ROS2 局域网自动发现配置

set -e

# 可通过传参覆盖，默认统一使用 Domain 10
ROS_DOMAIN_ID="${1:-10}"

# 清理 Discovery Server 相关变量，切回 ROS2 原生局域网发现
unset ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE

export ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
export ROS_LOCALHOST_ONLY=0

if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
    ros2 daemon start >/dev/null 2>&1 || true
fi

echo "✅ ROS2 局域网发现模式已启用"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"
echo "sudo ufw disable"