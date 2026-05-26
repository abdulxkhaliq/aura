
import os
from typing import List

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    package_name = 'aura_description'

    pkg_share = get_package_share_directory(package_name)
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'so_arm101_display.rviz')
    xacro_file = os.path.join(pkg_share, 'urdf', 'so_arm101.urdf.xacro')

    robot_description_raw = xacro.process_file(xacro_file).toxml()

    rviz_config = LaunchConfiguration('rviz_config')
    run_rviz = LaunchConfiguration('run_rviz')
    standalone = LaunchConfiguration('standalone')

    launch_args: List[DeclareLaunchArgument] = [
        DeclareLaunchArgument(
            name='rviz_config',
            default_value=default_rviz_config_path,
            description='Path to RViz config file'
        ),
        DeclareLaunchArgument(
            name='run_rviz',
            default_value='false',
            description=(
                'Launch RViz and joint GUI locally. Default is false for headless robot operation. '
                'When false, run RViz on remote PC with: ros2 run rviz2 rviz2 '
                'and joint GUI with: ros2 run joint_state_publisher_gui joint_state_publisher_gui '
                'And then load the RViz config file from RViz (rviz/so_arm101_display.rviz).'
            )
        ),
        DeclareLaunchArgument(
            name='standalone',
            default_value='true',
            description=(
                'True (default): the GUI sliders publish directly to /joint_states, so RViz '
                'shows the URDF being driven by the sliders — visualization only, no real arm. '
                'False: the GUI publishes to /gui_joint_states so twin_teleop_node can bridge '
                'them to joint_commands — slider teleop into the real arm. Set False when you '
                'also have a servo_driver running that publishes /joint_states from the real arm '
                'so RViz mirrors the physical pose.'
            )
        )
    ]

    nodes: List[Node] = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(AndSubstitution(run_rviz, standalone)),
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(AndSubstitution(run_rviz, NotSubstitution(standalone))),
            remappings=[('joint_states', 'gui_joint_states')]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(run_rviz)
        ),
        LogInfo(
            msg=(
                '\n'
                '========================================\n'
                'RViz is configured to run remotely for visualization.\n'
                'Run on remote PC:\n'
                '   ros2 run joint_state_publisher_gui joint_state_publisher_gui &\n'
                '   ros2 run rviz2 rviz2 -d <path_to>/so_arm101_display.rviz\n'
                '========================================\n'
            ),
            condition=UnlessCondition(run_rviz)
        )
    ]

    return LaunchDescription(launch_args + nodes)
