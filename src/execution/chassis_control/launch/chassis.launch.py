#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 这里添加启动节点的配置，例如:
        # Node(
        #     package='chassis_control',
        #     executable='chassis_driver',
        #     name='chassis_driver_node'
        # )
    ])
