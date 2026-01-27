import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetRemap
from launch.actions import GroupAction

def generate_launch_description():
    # Path to the orbbec_camera launch file
    orbbec_camera_pkg_prefix = FindPackageShare('orbbec_camera')
    orbbec_launch_file = PathJoinSubstitution([orbbec_camera_pkg_prefix, 'launch', 'orbbec_camera.launch.py'])

    # Include the orbbec_camera launch file with remapping
    # We wrap it in a GroupAction to apply remappings only to this include if needed, 
    # but SetRemap in the main LD works too. GroupAction is cleaner.
    
    orbbec_camera_node = GroupAction(
        actions=[
            # Remap the topic from standard orbbec output to the requested /image_raw
            # The default topic is /camera/color/image_raw (assuming camera_name='camera')
            SetRemap(src='/camera/color/image_raw', dst='/image_raw'),
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([orbbec_launch_file]),
                launch_arguments={
                    'camera_name': 'camera',
                    'depth_registration': 'false',
                    'enable_point_cloud': 'false',
                    'enable_colored_point_cloud': 'false',
                    'enable_d2c_viewer': 'false',
                    'enable_color': 'true',
                    'enable_depth': 'false',
                    # Optimization for RDK X5: disable what's not needed
                }.items()
            )
        ]
    )

    return LaunchDescription([
        orbbec_camera_node
    ])
