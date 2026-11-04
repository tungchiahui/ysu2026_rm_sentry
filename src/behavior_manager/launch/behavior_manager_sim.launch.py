"""复用通用入口，连接已启动的 Gazebo 和仿真导航。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("behavior_manager")

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="red_standard_robot1",
            description="Robot namespace in Gazebo",
        ),
        DeclareLaunchArgument(
            "bt_xml_file",
            default_value=PathJoinSubstitution([
                package_share,
                "behavior_trees",
                "spin_test.xml",
            ]),
            description="Behavior tree XML file to execute",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    package_share,
                    "launch",
                    "behavior_manager.launch.py",
                ])
            ),
            launch_arguments={
                "namespace": LaunchConfiguration("namespace"),
                "use_sim_time": "true",
                "bt_xml_file": LaunchConfiguration("bt_xml_file"),
            }.items(),
        ),
    ])
