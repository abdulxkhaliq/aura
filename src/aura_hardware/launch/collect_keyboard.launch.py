from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    args = [
        DeclareLaunchArgument('front_device', default_value='0',
                              description='v4l2 index for the front (Lenovo FHD) camera'),
        DeclareLaunchArgument('wrist_device', default_value='6',
                              description='v4l2 index for the wrist (HD camera) camera'),
        DeclareLaunchArgument('speed',     default_value='0.5',
                              description='Keyboard teleop speed (rad/s)'),
        DeclareLaunchArgument('fast_mult', default_value='2.0',
                              description='Shift-key speed multiplier'),
    ]

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

    front_cam = Node(
        package='aura_hardware',
        executable='camera_node',
        name='front_camera',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('front_device'),
            'fps':    30, 'width':  640, 'height': 480,
        }],
        remappings=[('camera/image_raw', '/front/image_raw')],
    )

    wrist_cam = Node(
        package='aura_hardware',
        executable='camera_node',
        name='wrist_camera',
        output='screen',
        parameters=[{
            'device': LaunchConfiguration('wrist_device'),
            'fps':    30, 'width':  640, 'height': 480,
        }],
        remappings=[('camera/image_raw', '/wrist/image_raw')],
    )

    teleop = Node(
        package='aura_hardware',
        executable='keyboard_teleop_node',
        name='keyboard_teleop',
        output='screen',
        parameters=[{
            'speed':     LaunchConfiguration('speed'),
            'fast_mult': LaunchConfiguration('fast_mult'),
            'rate':      30.0,
        }],
    )

    return LaunchDescription(args + [servo, front_cam, wrist_cam, teleop])
