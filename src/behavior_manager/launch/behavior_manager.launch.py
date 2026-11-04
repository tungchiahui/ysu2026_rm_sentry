"""启动能力服务端和上层行为树，供实车与仿真共用。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool
    )
    bt_xml_file = ParameterValue(
        LaunchConfiguration("bt_xml_file"), value_type=str
    )

    # 后续能力 Action 服务端统一添加到这里，启动后应等待 Goal。
    # 当前云台扫描会自动执行，完成 Action 改造后再加入。
    servers = [
        Node(
            package="behavior_manager",
            executable="chassis_spin_server",
            name="chassis_spin_server",
            namespace=namespace,
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ]

    bt_node = Node(
        package="behavior_manager",
        executable="behavior_manager_node",
        name="behavior_manager",
        namespace=namespace,
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "bt_xml_file": bt_xml_file,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Namespace shared by the robot, servers and BT executor",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the simulation clock",
        ),
        DeclareLaunchArgument(
            "bt_xml_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("behavior_manager"),
                "behavior_trees",
                "spin_test.xml",
            ]),
            description="Behavior tree XML file to execute",
        ),
        *servers,
        bt_node,
    ])
