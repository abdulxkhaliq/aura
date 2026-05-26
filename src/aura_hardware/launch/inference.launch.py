import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_CKPT = str(Path.home() / 'aura_ws' / 'act' / 'checkpoints' / 'best.pth')

def generate_launch_description():
    checkpoint_arg = DeclareLaunchArgument(
        'checkpoint', default_value=DEFAULT_CKPT,
        description='Path to the ACT checkpoint .pth file')

    camera_device_arg = DeclareLaunchArgument(
        'camera_device', default_value='2',
        description='Video device index (2 = /dev/video2)')

    servo = Node(
        package='aura_hardware',
        executable='servo_driver',
        name='servo_driver',
        output='screen',
        parameters=[{
            'port':     '/dev/ttyACM0',
            'baudrate': 1000000,
            'torque':   True,
        }],
    )

    camera = Node(
        package='aura_hardware',
        executable='camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('camera_device'),
            'fps':    30,
            'width':  640,
            'height': 480,
        }],
    )

    inference = Node(
        package='aura_hardware',
        executable='inference_node',
        name='inference_node',
        output='screen',
        parameters=[{
            'checkpoint': LaunchConfiguration('checkpoint'),
        }],
    )

    return LaunchDescription([
        checkpoint_arg,
        camera_device_arg,
        servo,
        camera,
        inference,
    ])
