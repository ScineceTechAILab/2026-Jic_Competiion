#!/bin/bash
# ==========================================
# RDK X5 ROS2 局域网自动发现配置脚本
# ==========================================

set -e

ROS_DOMAIN_ID="${1:-10}"

unset ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE

export ROS_DOMAIN_ID="$ROS_DOMAIN_ID"
export ROS_LOCALHOST_ONLY=0

if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 || true
    ros2 daemon start >/dev/null 2>&1 || true
fi

echo "✅ RDK X5 局域网发现配置完成"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY=$ROS_LOCALHOST_ONLY"
echo "RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"