from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
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

    return LaunchDescription([servo])
