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
        DeclareLaunchArgument('gripper_speed', default_value='0.5',
                              description='Gripper keyboard speed (rad/s)'),
        DeclareLaunchArgument('gripper_fast_mult', default_value='2.0',
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
            'torque':   False,
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

    gripper_teleop = Node(
        package='aura_hardware',
        executable='keyboard_teleop_node',
        name='gripper_keyboard',
        output='screen',
        parameters=[{
            'mode':      'gripper_only',
            'speed':     LaunchConfiguration('gripper_speed'),
            'fast_mult': LaunchConfiguration('gripper_fast_mult'),
            'rate':      30.0,
        }],
    )

    return LaunchDescription(args + [servo, front_cam, wrist_cam, gripper_teleop])
