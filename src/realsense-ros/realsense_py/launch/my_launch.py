import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): 

    return LaunchDescription([
        Node(
            package='realsense_py', 
            executable='cam2cam_cal',
            name='Camera2CameraCalibration',
            output='screen', 
            emulate_tty=True,
        )
    ])

