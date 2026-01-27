#!/bin/bash
# Install dependencies for OrbbecSDK_ROS2
# Reference: https://github.com/orbbec/OrbbecSDK_ROS2

echo "Installing OrbbecSDK_ROS2 dependencies..."
sudo apt-get update
sudo apt-get install -y libgflags-dev nlohmann-json3-dev \
ros-humble-image-transport ros-humble-image-transport-plugins ros-humble-compressed-image-transport \
ros-humble-image-publisher ros-humble-camera-info-manager \
ros-humble-diagnostic-updater ros-humble-diagnostic-msgs ros-humble-statistics-msgs ros-humble-xacro \
ros-humble-backward-ros libdw-dev libssl-dev mesa-utils libgl1 libgoogle-glog-dev

echo "Dependencies installed."

echo "Cloning sllidar_ros2..."
if [ ! -d "src/support/sdk/sllidar_ros2" ]; then
    git clone https://github.com/Slamtec/sllidar_ros2.git src/support/sdk/sllidar_ros2
else
    echo "sllidar_ros2 already exists."
fi
