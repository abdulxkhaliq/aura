import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='2',
        description='Video device index (2 = /dev/video2, Lenovo FHD Webcam)',
    )

    servo = Node(
        package='aura_hardware',
        executable='servo_driver',
        name='servo_driver',
        output='screen',
        parameters=[{
            'port':     '/dev/ttyACM0',
            'baudrate': 1000000,
            'torque':   False,
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

    return LaunchDescription([
        camera_device_arg,
        servo,
        camera,
    ])
