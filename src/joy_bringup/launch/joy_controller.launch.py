import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    joy_launch_file = os.path.join(
        get_package_share_directory('joy_bringup'),
        'launch',
        'joy.launch.py',
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(joy_launch_file),
        ),
        Node(
            package='joy_bringup',
            executable='joy_bringup',
            name='joy_node_cpp',
            output='screen',
        ),
    ])
